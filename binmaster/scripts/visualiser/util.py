import numpy as np
import rpy2.robjects as ro
from rpy2.rinterface_lib import callbacks
from rpy2.robjects import FloatVector
from rpy2.robjects.packages import importr
from scipy.signal import butter, filtfilt

importr('changepoint')
callbacks.consolewrite_warnerror = lambda *args: None


def butter_lowpass_filter(data, cutoff, fs, order):
    normal_cutoff = cutoff / (0.5 * fs)
    # Get the filter coefficients
    b, a = butter(order, normal_cutoff, btype='low', analog=False, output='ba')
    y = filtfilt(b, a, data)
    return y


def calculate_position(log, position, count_to_mm=1.91):
    encoder_ticks = sum(log['count'][:position])
    return encoder_ticks * count_to_mm


def detect_bottom(log, count_to_mm=1.91, initial_offset=2500, cutoff=70):
    count = 0
    index = initial_offset
    for val in log['count'][initial_offset:]:
        index += 1

        if val:
            count = 0
        else:
            count += 1

        if count > cutoff:
            break

    encoder_ticks = sum(log['count'][:index])
    bottom_distance = encoder_ticks * count_to_mm

    return {'index': index, 'value': log['tension'][index], 'depth': bottom_distance}


def detect_water(log, count_to_mm=1.91, initial_offset=2500):
    bottom_peak_index = np.argmin(log['tension_low'])
    if bottom_peak_index > (4 + initial_offset):
        tension = FloatVector(log['tension_low'][initial_offset:bottom_peak_index])
        change_points = ro.r['cpt.meanvar'](tension)
        indexes = [int(x) + 2000 for x in list(ro.r['cpts'](change_points))]

        if len(indexes) == 1:
            encoder_ticks = sum(log['count'][:indexes[0]])
            water_depth = encoder_ticks * count_to_mm

            return {'index': indexes[0], 'value': log['tension_low'][indexes[0]], 'level': water_depth}
        elif len(indexes) > 1:
            raise ValueError('Located to many change points')
        else:
            raise ValueError('Unable to locate change points')
    else:
        raise ValueError('Unable to locate bottom peak')
