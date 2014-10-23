import math


GAUSSIAN_SMOOTH_SDEV = float(1.0)
GAUSSIAN_SMOOT_SECONDARY_DATA = float(2.0)
GAUSSIAN_SMOOT_TERTIARY_DATA = float(2.5)


MAX_FILTER_WIDTH = 31


def ApplyFilter(data, filter, filter_sum):
    mid = len(filter) / 2
    new_data = [0] * len(data)
    for i in range(len(data)):
        if i < mid or i > len(data) - mid:
            new_data[i] = data[i]
        else:
            sum = 0
            #count = 0
            for j in range(-mid, mid + 1):
                idx = i + j
                if idx >= 0 and idx < len(data):
                    sum += data[idx] * filter[mid + j]
                #count += 1
                new_data[i] = sum / filter_sum# / count
    return new_data

def BuildGaussianFilter(width, sdev):
    if width > MAX_FILTER_WIDTH:
        width = MAX_FILTER_WIDTH
    mid = width / 2
    filter = [0] * width
    for x in range(-mid, mid + 1):
        filter[mid + x] = 1 / math.sqrt(2 * math.pi * sdev) * math.exp(-(x * x) / (2 * sdev * sdev))
    return filter
                
def SmoothData(dataArray, sdev):
    filter = BuildGaussianFilter(2 * ((int(sdev) * 10) / 2) + 1, sdev)#[1] * width
    filter_sum = 0
    for value in filter:
        filter_sum += value
    return ApplyFilter(dataArray, filter, filter_sum)
