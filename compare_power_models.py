import sys
import math
import tempfile
import Gnuplot


def wait(str=None, prompt='Press return to show results...\n'):
    if str is not None:
        print str
    raw_input(prompt)


def ExtractValueFromLine(line):
    parts = line.split('=')
    return float(parts[1].strip())


def ParseFile(fileName):
    with open(fileName, 'r') as f:
        lines = f.readlines()
    
    lines = [item.strip() for item in lines]
    
    model1Values = []
    model2Values = []

    i = 0
    while i < len(lines):
        if lines[i] != 'Lap':
            print 'Invalid file format: "%s"' % line[i]
            return None
        model1Values.append(ExtractValueFromLine(lines[i+1]))
        model2Values.append(ExtractValueFromLine(lines[i+2]))
        i += 3
    return (model1Values, model2Values)


def PlotValues(values1, values2):
    fileName1 = tempfile.mktemp()
    with open(fileName1, 'w') as f:
        for value in values1:
            f.write('%s\n' % value)
    
    fileName2 = tempfile.mktemp()
    with open(fileName2, 'w') as f:
        for value in values2:
            f.write('%s\n' % value)
            
    g = Gnuplot.Gnuplot(debug=1)
    g.clear()
    g.plot(Gnuplot.File(fileName1, with_='lines'), Gnuplot.File(fileName2, with_='lines'))
    wait('Press Enter')
    
class Stats:
    def __init__(self, values, mean, sdev, minimum, maximum):
        self.mean = mean
        self.sdev =sdev
        self.min = minimum
        self.max = maximum
        self.sortedValues = sorted(values)
        self.median = self.GetPercentileValue(50)
    
    def Print(self):
        print '  mean = %f, sdev = %f, min = %f, max = %f' % (self.mean, self.sdev, self.min, self.max)
        print '  median = %f, 10%% = %f, 90%% = %f' % (self.mean, self.GetPercentileValue(10), self.GetPercentileValue(90))
    
    def PrintPercentiles(self, step):
        for p in range(0, 100, step):
            print '    %d%% = %f' % (p, self.GetPercentileValue(p))
    
    def PlotPercentiles(self):
        fileName = tempfile.mktemp()
        with open(fileName, 'w') as f:
            for value in self.sortedValues:
                f.write('%s\n' % value)
        g = Gnuplot.Gnuplot(debug=1)
        g.clear()
        g.plot(Gnuplot.File(fileName, with_='lines'))
        wait('Press Enter')
        
    
    def GetPercentileValue(self, percentile):
        return self.sortedValues[percentile * len(self.sortedValues) / 100]
    
    @staticmethod
    def Generate(values):
        meanAccum = 0
        sdevAccum = 0
        minimum = None
        maximum = None
        n = len(values)
        for value in values:
            meanAccum += value
            if not minimum or value < minimum:
                minimum = value
            if not maximum or value > maximum:
                maximum = value
        
        mean = meanAccum / n

        for value in values:
            diff = value - mean
            sdevAccum += diff * diff
        
        sdev = math.sqrt(sdevAccum / n)
        
        return Stats(values, mean, sdev, minimum, maximum)

class Range:
    def __init__(self, minimum, maximum):
        self.min = minimum
        self.max = maximum
    
    def InRange(self, value):
        return value >= self.min and value <= self.max

def ThrowOutOutliers(values, stats):
    sdevRange = Range(stats.mean - stats.sdev, stats.mean + stats.sdev)
    print 'Removing all values outside of range %f - %f' % (sdevRange.min, sdevRange.max)
    #print values
    newValues = [value for value in values if sdevRange.InRange(value)]
    #print newValues
    return newValues
    
def ProcessCamparisonFile(fileName):
    values = ParseFile(fileName) 
    if not values:
        print 'Error parsing "%s"' % fileName
    model1Values, model2Values = values
    
    stats1 = Stats.Generate(model1Values)
    print 'Model 1 Stats:'
    stats1.Print()
    model1Values = ThrowOutOutliers(model1Values, stats1)            
    stats1 = Stats.Generate(model1Values)
    model1Values = ThrowOutOutliers(model1Values, stats1)            
    stats1 = Stats.Generate(model1Values)
    stats1.Print()
    #stats1.PlotPercentiles()            
    
    stats2 = Stats.Generate(model2Values)
    print 'Model 2 Stats:'
    stats2.Print()
    model2Values = ThrowOutOutliers(model2Values, stats2)            
    stats2 = Stats.Generate(model2Values)
    model2Values = ThrowOutOutliers(model2Values, stats2)            
    stats2 = Stats.Generate(model2Values)
    stats2.Print()            
    #stats2.PrintPercentiles(5)            
    #stats2.PlotPercentiles()
    
    PlotValues(stats1.sortedValues, stats2.sortedValues)            

def Main(programName, args):
    if len(args) < 1:
        print 'You must specify and input file name'
        return -1
    
    fileName = args[0]
    ProcessCamparisonFile(fileName)
    return 0

if __name__ == '__main__':
    sys.exit(Main(sys.argv[0], sys.argv[1:]))