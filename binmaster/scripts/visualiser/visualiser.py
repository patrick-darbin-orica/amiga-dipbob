import os
import json
import dash
import sqlite3
import argparse
import threading
import subprocess
import dash_table as table
import dash_core_components as dcc
import dash_html_components as html
import plotly.io as io
import plotly.graph_objs as go
from pathlib import Path
from flask import Flask
from dash.exceptions import PreventUpdate
from dash.dependencies import Input, Output, State
import can
import threading
from protocol import Protocol, PhysicalLayer
from util import butter_lowpass_filter, detect_bottom, detect_water


last_cycle = {'tension': [], 'count': []}

# Added for CAN interface ###################
def can_listener(bus_channel='can0', can_id=0x12345678):
    """Listen for CAN messages on specific channel and trigger a cycle on receiving the correct signal"""
    bus = can.interface.Bus(channel = bus_channel, bustype = 'socketcan')
    print(f"Listening for CAN messages on {bus_channel}...")

    while True:
        message = bus.recv() # Blocking call; wait for message
        if message.arbitration_id == can_id and message.data[0] == 0x01:
            add_log("Started Cycle")
            
            defaults = get_defaults_database(True)
            uid = add_cycle(0, 0, defaults['ascent_rate'], defaults['descent_rate'],
                            defaults['timeout_0'], defaults['timeout_1'], defaults['timeout_2'],
                            defaults['timeout_3'], defaults['ramp_rate'])
            threading.Thread(target=perform_cycle, args=(uid,)).start()
##################################################

def build_layout():
    """ Builds HTML layout for UI

    Returns:
        Layout Structure
    """
    return html.Div(
        id='graph_div',
        children=[
            dcc.Graph(id='graph', style={'height': '70vh'}),
            dcc.Interval(
                id='interval',
                interval=5000,  # 5s
                n_intervals=0
            ),
            html.Div(children=[
                html.Div(
                    id='text_div',
                    children=[
                        dcc.Textarea(id='text', style={'height': '20vh', 'width': '100%'})
                    ],
                    className="three columns"
                ),
                html.Div(
                    id='history_div',
                    children=[
                        table.DataTable(
                            id='cycles-table',
                            columns=(
                                [{'id': 'date', 'name': 'Date'}, {'id': 'ascent_rate', 'name': 'Ascent Rate'},
                                 {'id': 'descent_rate', 'name': 'Descent Rate'},
                                 {'id': 'timeout_0', 'name': 'Timeout 0'}, {'id': 'timeout_1', 'name': 'Timeout 1'},
                                 {'id': 'timeout_2', 'name': 'Timeout 2'}, {'id': 'timeout_3', 'name': 'Timeout 3'},
                                 {'id': 'ramp_rate', 'name': 'Ramp Rate'},
                                 {'id': 'link', 'name': 'Link', 'presentation': 'markdown'}]
                            ),
                            data=get_cycles(),
                            editable=False
                        )
                    ],
                    className="five columns"
                ),
                html.Div(
                    id='table_div',
                    children=[
                        table.DataTable(
                            id='parameters-table',
                            columns=(
                                [{'id': 'ascent_rate', 'name': 'Ascent Rate'},
                                 {'id': 'descent_rate', 'name': 'Descent Rate'},
                                 {'id': 'timeout_0', 'name': 'Timeout 0'},
                                 {'id': 'timeout_1', 'name': 'Timeout 1'},
                                 {'id': 'timeout_2', 'name': 'Timeout 2'},
                                 {'id': 'timeout_3', 'name': 'Timeout 3'},
                                 {'id': 'ramp_rate', 'name': 'Ramp Rate'}]
                            ),
                            data=[get_defaults_database(True)],
                            editable=True
                        ),
                        html.Br(),
                        html.Button('RESET', id='reset'),
                        html.Br(),
                        html.Button('CYCLE', id='cycle'),
                    ],
                    className="four columns"
                )
            ], className="row"),
            html.Div(id='hidden-cycle', style={'display': 'none'}),
            html.Div(id='hidden-edit', style={'display': 'none'})
        ]
    )


def get_logs():
    """ Retrieve all logs from SQLite database and format into string

    Returns:
        String of logs
    """
    conn = sqlite3.connect(str(Path(server.root_path) / 'assets' / 'logging.db'))
    c = conn.cursor()
    results = c.execute("SELECT * FROM log").fetchall()

    text = ""
    for row in results:
        text = "{}: {}\n".format(row[0], row[1]) + text

    return text


def add_log(entry: str):
    """ Insert a new log entry into the database

    Args:
        entry (str): Log entry to insert into database
    """
    conn = sqlite3.connect(str(Path(server.root_path) / 'assets' / 'logging.db'))
    c = conn.cursor()
    c.execute("INSERT INTO `log` (entry) VALUES (?)", (entry,))
    conn.commit()
    conn.close()


def get_cycles():
    """ Retrieve parameters about all dipping cycles formatted for tabulating in UI

    Returns:
        List of dicts containing cycle parameters
    """
    conn = sqlite3.connect(str(Path(server.root_path) / 'assets' / 'logging.db'))
    c = conn.cursor()

    results = c.execute("SELECT date, lat, lon, ascent_rate, descent_rate, timeout_0, timeout_1, timeout_2, timeout_3, "
                        "ramp_rate, cycles.id FROM cycles "
                        "JOIN cycle_data ON cycles.id = cycle_data.id "
                        "JOIN cycle_properties ON cycles.id = cycle_properties.id "
                        "ORDER BY cycles.id DESC").fetchall()

    rows = []
    for result in results:
        rows.append({'date': result[0], 'lat': result[1], 'lon': result[2], 'ascent_rate': result[3],
                     'descent_rate': result[4], 'timeout_0': result[5], 'timeout_1': result[6], 'timeout_2': result[7],
                     'timeout_3': result[8], 'ramp_rate': result[9], 'link': '[{0}](/graph/{0})'.format(result[10])})

    return rows


def add_cycle(lat: float, lon: float, ascent_rate: int, descent_rate: int, timeout_0: int, timeout_1: int,
              timeout_2: int, timeout_3: int, ramp_rate: int):
    """ Add new cycle parameters to database

    Args:
        lat (float): Latitude at which cycle occurred
        lon (float): Longitude at which cycle occurred
        ascent_rate (int): Duty cycle of winch in ascent
        descent_rate (int): Duty cycle of winch in descent
        timeout_0 (int): Initial descent timeout
        timeout_1 (int): Descent timeout
        timeout_2 (int): Initial ascent timeout
        timeout_3 (int): Ascent timeout
        ramp_rate (int): Ramp rate of soft start

    Returns:
        Unique ID of cycle from DB
    """
    conn = sqlite3.connect(str(Path(server.root_path) / 'assets' / 'logging.db'))
    c = conn.cursor()

    # Time and Location of Cycle
    c.execute("INSERT INTO cycles (lat, lon) VALUES (?, ?)", (lat, lon))
    # Get UID assigned by SQLite
    uid = c.execute("SELECT id FROM cycles ORDER BY id DESC LIMIT 1").fetchone()[0]
    # Record where tension log is saved
    c.execute("INSERT INTO cycle_data (id, file) VALUES (?, ?)", (uid, '{}.json'.format(uid)))
    # Record parameters of cycle
    c.execute("INSERT INTO cycle_properties (id, ascent_rate, descent_rate, timeout_0, timeout_1, timeout_2, "
              "timeout_3, ramp_rate) VALUES (?, ?, ?, ?, ?, ?, ?, ?)",
              (uid, ascent_rate, descent_rate, timeout_0, timeout_1, timeout_2, timeout_3, ramp_rate))

    conn.commit()
    conn.close()
    return uid


def get_defaults_database(live: bool):
    """ Get either the current values or the default tuning values from the database 

    Args:
        live (bool): Get live values if True else get default values

    Returns:
        dict of parameters
    """
    conn = sqlite3.connect(str(Path(server.root_path) / 'assets' / 'logging.db'))
    c = conn.cursor()
    results = c.execute("SELECT ascent_rate, descent_rate, timeout_0, timeout_1, timeout_2, timeout_3, ramp_rate "
                        "FROM defaults WHERE live = ?", (int(live),)).fetchone()
    conn.close()
    return {'ascent_rate': results[0], 'descent_rate': results[1], 'timeout_0': results[2], 'timeout_1': results[3],
            'timeout_2': results[4], 'timeout_3': results[5], 'ramp_rate': results[6]}


def set_defaults_database(ascent_rate: int, descent_rate: int, timeout_0: int, timeout_1: int,
                          timeout_2: int, timeout_3: int, ramp_rate: int):
    """ Save current parameters to database

    Args:
        ascent_rate (int): Duty cycle of winch in ascent
        descent_rate (int): Duty cycle of winch in descent
        timeout_0 (int): Initial descent timeout
        timeout_1 (int): Descent timeout
        timeout_2 (int): Initial ascent timeout
        timeout_3 (int): Ascent timeout
        ramp_rate (int): Ramp rate of soft start
    """
    conn = sqlite3.connect(str(Path(server.root_path) / 'assets' / 'logging.db'))
    c = conn.cursor()
    c.execute("UPDATE defaults SET ascent_rate = ?, descent_rate = ?, timeout_0 = ?, timeout_1 = ?, timeout_2 = ?, "
              "timeout_3 = ?, ramp_rate = ? WHERE live = 1",
              (ascent_rate, descent_rate, timeout_0, timeout_1, timeout_2, timeout_3, ramp_rate))
    conn.commit()
    conn.close()


def perform_cycle(uid):
    """ Spawn new process and run a cycle of the dipper. Will save to a json log file.

    Args:
        uid: uid from Database, used to name the log file
    """
    # Use global variable to save data for UI update
    global last_cycle
    # Spawn new process that captures encoder streams and saves to log
    subprocess.Popen('python3 cycle.py -f logs/{}.json'.format(uid), shell=True, cwd=os.getcwd()).wait()
    # Read log into global variable
    last_cycle = get_log(uid)
    add_log("Cycle Complete")


def graph(log):
    """ Graph a particular cycle

    Args:
        log (dict): Dictionary of encoder and analysis information about the cycle to be graphed

    Returns:
        Plotly Figure
    """
    # Setup Graph
    fig = go.Figure()

    try:
        # Graph Distance
        fig.add_trace(go.Scatter(y=log['count'], name='Count'))

        # Graph Tension
        fig.add_trace(go.Scatter(y=log['tension'], name='Tension'))

        # Graph Filtered
        fig.add_trace(go.Scatter(y=log['tension_low'], name='Low Pass'))

        # Graph Bottom
        fig.add_trace(go.Scatter(x=[log['bottom_stats']['index']],
                                 y=[log['bottom_stats']['value']],
                                 hovertext=['{}mm'.format(log['bottom_stats']['depth'])],
                                 mode='markers',
                                 marker=dict(symbol='x', size=15),
                                 name='Bottom'))

        # Graph Water
        fig.add_trace(go.Scatter(x=[log['water_stats']['index']],
                                 y=[log['water_stats']['value']],
                                 hovertext=['{}mm'.format(log['bottom_stats']['depth'] - log['water_stats']['level'])],
                                 mode='markers',
                                 marker=dict(symbol='x', size=15),
                                 name='Water'))
    except KeyError:
        # If analysis is not running, (-ng flag) bottom and water may not be present in dict
        pass
    return fig


# Create Flask server
server = Flask(__name__)

# Spawn Dash application on Flask server
app = dash.Dash(server=server)

# Specify name and layout of UI
app.title = 'Dip and Bob Visualiser'
app.layout = build_layout


@app.callback(Output('text', 'value'),
              [Input('interval', 'n_intervals')])
def update_console(n):
    """ Retrieve console log entries from database and insert into UI

    Runs on polled interval from user's browser

    Args:
        n:

    Returns: Log entries as string

    """
    return get_logs()


@app.callback(Output('cycles-table', 'data'),
              [Input('interval', 'n_intervals')])
def update_cycles(n):
    """ Retrieve cycle information from database and insert into UI

    Runs on polled interval from user's browser

    Args:
        n:

    Returns: Cycle details as as dict

    """
    return get_cycles()


@app.callback(Output('graph', 'figure'),
              [Input('interval', 'n_intervals')])
def live_graph(n):
    """ Update the graph in the UI with latest cycle (if -ng flag not present)

    Runs on polled interval from user's browser

    Args:
        n:

    Returns:
        Plotly figure
    """
    if args.graph:
        # If graphing and analysis are running
        global last_cycle
        return graph(last_cycle)
    else:
        # Return nothing
        raise PreventUpdate


@app.callback(Output('hidden-cycle', 'children'),
              [Input('cycle', 'n_clicks')],
              [State('parameters-table', 'data')])
def cycle_clicked(n, rows):
    """ Run a cycle when user clicks button in UI

    Args:
        n:
        rows (list): Current parameters from the UI
    """
    if n is not None:
        add_log("Started Cycle")
        parameters = rows[0]
        uid = add_cycle(0, 0, int(parameters['ascent_rate']), int(parameters['descent_rate']),
                        int(parameters['timeout_0']), int(parameters['timeout_1']), int(parameters['timeout_2']),
                        int(parameters['timeout_3']), int(parameters['ramp_rate']))
        threading.Thread(target=perform_cycle, args=(uid,)).start()
    raise PreventUpdate  # Prevent UI from throwing errors as function does not return anything


@app.callback(Output('parameters-table', 'data'),
              [Input('reset', 'n_clicks')])
def reset_clicked(n):
    """ Reset the table in UI to default parameters

    Args:
        n:

    Returns:
        List with default values for table in UI
    """
    if n is not None:
        return [get_defaults_database(False)]
    raise PreventUpdate  # Prevent UI from throwing errors as function does not return anything


@app.callback(Output('hidden-edit', 'children'),
              [Input('parameters-table', 'data')])
def update_parameters(rows):
    """ Update database with new values if user edits parameters table

    Args:
        rows (list): Current parameters from the UI
    """
    global mac
    if rows:
        parameters = rows[0]

        set_defaults_database(parameters['ascent_rate'], parameters['descent_rate'], parameters['timeout_0'],
                              parameters['timeout_1'], parameters['timeout_2'], parameters['timeout_3'],
                              parameters['ramp_rate'])

        mac.execute_set_ascent_rate(int(parameters['ascent_rate']))
        mac.execute_set_descent_rate(int(parameters['descent_rate']))
        mac.execute_set_timeout(0, int(parameters['timeout_0']))
        mac.execute_set_timeout(1, int(parameters['timeout_1']))
        mac.execute_set_timeout(2, int(parameters['timeout_2']))
        mac.execute_set_timeout(3, int(parameters['timeout_3']))
        mac.execute_set_ramp_rate(int(parameters['ramp_rate']))

        add_log("Updated Parameters: {}".format(rows[0]))
    raise PreventUpdate


@server.route('/logs/<int:uid>', methods=['GET'])
def get_log(uid):
    """ Read json formatted log file from specified cycle and perform analysis (if -ng not present)

    Also mapped as a url which will return JSON to browser

    Args:
        uid: Unique ID of log to be loaded

    Returns:
        dict of cycle data
    """
    with open("logs/{}.json".format(uid)) as f:
        log = json.load(f)
        log['tension_low'] = butter_lowpass_filter(log['tension'], 3, 5e3, 1)
        if args.graph:
            log['bottom_stats'] = detect_bottom(log)
            try:
                log['water_stats'] = detect_water(log)
            except ValueError:
                add_log("Water Detection Error")
    return log


@server.route('/graph/<int:uid>', methods=['GET'])
def get_graph(uid):
    """ Access graph of particular cycle via URL

    Args:
        uid: Unique ID of cycle to be graphed

    Returns:
        Plotly graph as webpage
    """
    log = get_log(uid)
    return io.to_html(graph(log))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Dip and Bob Visualiser')
    parser.add_argument('-b', '--baud', help='Baudrate', default=1000000, dest='baud', type=int)
    parser.add_argument('-p', '--port', help='Serial Port',
                        default='/dev/ttyACM0', dest='port')
    parser.add_argument('-ng', '--no-graphs', help='Will disable immediate analysis and graphing, helps speed'
                                                   'up processing on long holes', action='store_false', dest='graph')
    args = parser.parse_args()

    # Start CAN message listener in a separate thread
    can_thread = threading.Thread(target=can_listener, kwargs={'bus_channel': 'vcan0', 'can_id': 0x123})
    can_thread.daemon = True  # Thread stops when the main program exits
    can_thread.start()

    # Setup serial port to control STM32 in control board
    phy = PhysicalLayer(baud=args.baud, port=args.port)
    mac = Protocol(phy)

    # Load defaults from database and send to STM32 in control board
    defaults = get_defaults_database(True)
    mac.execute_set_ascent_rate(defaults['ascent_rate'])
    mac.execute_set_descent_rate(defaults['descent_rate'])
    mac.execute_set_timeout(0, defaults['timeout_0'])
    mac.execute_set_timeout(1, defaults['timeout_1'])
    mac.execute_set_timeout(2, defaults['timeout_2'])
    mac.execute_set_timeout(3, defaults['timeout_3'])
    mac.execute_set_ramp_rate(defaults['ramp_rate'])

    # Run Dash UI
    app.run_server(debug=False, use_reloader=False, host='0.0.0.0', port = 8050, dev_tools_silence_routes_logging=True)

