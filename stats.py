import math


def GetValueAtTime(values, times, t):
    for i in range(len(values)):
        if t >= times[i]:
            if t == times[i] or i == len(times) - 1 or t > times[i+1]:
                return values[i]
            timeSpan = float(times[i+1] - times[i])
            valueSpan = float(values[i+1] - values[i])
            return values[i] + valueSpan * (t - times[i]) / timeSpan
    return 0

def Mean(data):
    sum = 0
    for d in data:
        sum += d
    return sum / len(data)

def Sdev(data):
    m = Mean(data)
    acum = 0
    for d in data:
        dif = d - m;
        acum += d * d
    return math.sqrt(acum / len(data))             

