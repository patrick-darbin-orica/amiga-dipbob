import json
import sqlite3
import argparse
from pathlib import Path
from multiprocessing import Pool
from util import butter_lowpass_filter, detect_bottom, detect_water


def get_logs():
    """ Get all cycles and associated log files from the database

    Returns:
        list(dict) of all uid's and associated log files of cycles
    """
    conn = sqlite3.connect(str(Path('assets/logging.db')))
    c = conn.cursor()

    results = c.execute("SELECT id, file FROM cycle_data").fetchall()

    rows = [{'id': result[0], 'file': result[1]} for result in results]
    return rows


def get_cycles():
    """ Get parameters about all cycles from the database

    Returns:
        list(dict) of parameters of all cycles
    """
    conn = sqlite3.connect(str(Path('assets/logging.db')))
    c = conn.cursor()

    results = c.execute("SELECT date, lat, lon, ascent_rate, descent_rate, timeout_0, timeout_1, timeout_2, timeout_3, "
                        "ramp_rate FROM cycles "
                        "JOIN cycle_data ON cycles.id = cycle_data.id "
                        "JOIN cycle_properties ON cycles.id = cycle_properties.id").fetchall()

    info = [{'date': result[0], 'lat': result[1], 'lon': result[2], 'ascent_rate': result[3],
             'descent_rate': result[4], 'timeout_0': result[5], 'timeout_1': result[6], 'timeout_2': result[7],
             'timeout_3': result[8], 'ramp_rate': result[9]} for result in results]

    return info


def compute_levels(file):
    """ Worker function to compute properties about depth and water level of a given log file

    Args:
        file (str): Filename of a log. Likely similar to 121.json

    Returns:
        dict: Depth and water level properties of particular log
    """
    try:
        with open("logs/{}".format(file)) as f:
            try:
                # Load log file from JSON
                log = json.load(f)

                # Apply lowpass filtering
                log['tension_low'] = butter_lowpass_filter(log['tension'], 3, 5e3, 1)

                # Compute Depth and Water Level
                hole_depth = detect_bottom(log)['depth']
                water_depth = detect_water(log)['level']

                stats = {'depth': hole_depth, 'water': water_depth}
                return stats
            except (IndexError, ValueError):
                print('{}: Detection Failure'.format(file))
                return None
    except FileNotFoundError:
        return None


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Dip and Bob Log Analyser')
    parser.add_argument('-p', '--processes', help='Number of process to use', default=8, type=int, dest='processes')

    args = parser.parse_args()

    # Get log files for analysis
    logs = [log['file'] for log in get_logs()]
    print(logs)

    # Launch pool of worker processes
    with Pool(processes=args.processes) as pool:
        # For each log, perform analysis and collate results
        depths = pool.map(compute_levels, logs)
        print(depths)

    # Get parameters for each log
    cycles = get_cycles()
    print(cycles)

    # Assemble into a csv file for output
    with open('log_scrub_v3.csv', 'w') as f:
        f.write('id, date, ascent_rate, descent_rate, timeout_0, timeout_1, timeout_2, timeout_3, '
                'ramp_rate, depth, water\n')
        for i in range(len(cycles)):
            try:
                f.write('{}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {}\n'.format(
                    i + 1, cycles[i]['date'], cycles[i]['ascent_rate'], cycles[i]['descent_rate'], cycles[i]['timeout_0'],
                    cycles[i]['timeout_1'], cycles[i]['timeout_2'], cycles[i]['timeout_3'], cycles[i]['ramp_rate'],
                    depths[i]['depth'], depths[i]['water']))
            except TypeError:
                f.write('{}, {}, {}, {}, {}, {}, {}, {}, {}, null, null\n'.format(
                    i + 1, cycles[i]['date'], cycles[i]['ascent_rate'], cycles[i]['descent_rate'], cycles[i]['timeout_0'],
                    cycles[i]['timeout_1'], cycles[i]['timeout_2'], cycles[i]['timeout_3'], cycles[i]['ramp_rate']))
