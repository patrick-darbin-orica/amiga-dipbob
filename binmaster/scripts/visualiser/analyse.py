import json
import argparse
import plotly.graph_objects as go
from util import butter_lowpass_filter, detect_bottom, detect_water, calculate_position

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Dip and Bob Log Analyser')
    parser.add_argument('-f', '--file', help='Log file to read from', default='temp.json', dest='file')
    args = parser.parse_args()

    with open(args.file) as f:
        log = json.load(f)

        # Find Stop
        bottom_stats = detect_bottom(log)
        print('Distance to bottom: {}mm'.format(bottom_stats['depth']))

        # Setup Graph
        fig = go.Figure()

        # Graph Distance
        fig.add_trace(go.Scatter(y=log['count'], name='Count'))

        # Graph Tension
        fig.add_trace(go.Scatter(y=log['tension'], name='Tension'))

        # GraphFilter
        log['tension_low'] = butter_lowpass_filter(log['tension'], 3, 5e3, 1)
        fig.add_trace(go.Scatter(y=log['tension_low'], name='Low Pass'))

        # Graph Bottom
        fig.add_trace(go.Scatter(x=[bottom_stats['index']],
                                 y=[bottom_stats['value']],
                                 hovertext=['{}mm'.format(bottom_stats['depth'])],
                                 mode='markers',
                                 marker=dict(symbol='x', size=15),
                                 name='Bottom'))

        # Find Water
        water_stats = detect_water(log)
        if (bottom_stats['depth'] - water_stats['level']) > 100:
            print('Water Depth: {}mm'.format(water_stats['level']))
            fig.add_trace(go.Scatter(x=[water_stats['index']],
                                     y=[water_stats['value']],
                                     hovertext=['{}mm'.format(bottom_stats['depth'] - water_stats['level'])],
                                     mode='markers',
                                     marker=dict(symbol='x', size=15),
                                     name='Water'))
        else:
            print('Detected water depth less than bob length, likely no water')

        fig.show()

        # Get Actual Position from user

        val = input('Enter index of water level: ')
        if val is not '':
            print('Water Position: {}'.format(calculate_position(log, int(val))))
