import Gnuplot
import tempfile
import numpy

def wait(str=None, prompt='Press return to show results...\n'):
    if str is not None:
        print str
    raw_input(prompt)


class PlotData:
    def __init__(self, title, xdata, ydata):
        self.title = title
        self.xdata = xdata
        self.ydata = ydata
        self.min = self.ydata[0]
        self.max = self.ydata[0]
        for value in ydata[1:]:
            if value > self.max:
                self.max = value
            if value < self.min:
                self.min = value
    
    def yRange(self):
        return self.max - self.min
    
    def IsMidLineVisible(self):
        #print '[%.4f - %.f4' % ()
        return self.min < 0 and self.max > 0

    def GetMidLine(self):
        return self.yResolution * (0 - self.min) / self.yRange()
    
    def ScaleData(self, yResolution):
        self.yResolution = yResolution
        # Always normalize the data to 0
        # Always scale normalized data from 0 to yResolution
        base = self.min
        yRange = self.yRange()
        if yRange == 0:
            yRange = yResolution
        newYData = [0] * len(self.ydata)
        for i in range(len(self.ydata)):
            newYData[i] = (self.ydata[i] - base) * yResolution / yRange
        self.ydata = newYData
    
    def WriteToTempFile(self):
        self.tempFileName = tempfile.mktemp()
        f = open(self.tempFileName, 'w')
        for i in range(len(self.xdata)):
            f.write('%s %s\n' % (self.xdata[i], self.ydata[i]))
        f.close()
        return self.tempFileName

class MuliPlotter:
    def __init__(self, yResolution):
        self.yResolution = yResolution
        self.plots = []
    
    def AddPlot(self, plot):
        plot.ScaleData(self.yResolution)
        self.plots.append(plot)
    
    def Plot(self):
        g = Gnuplot.Gnuplot()
        g.clear()
        plots = []
        color = 1
        g.xlabel('time (seconds)')
        for plot in self.plots:
            fn = plot.WriteToTempFile()
            g.ylabel(plot.title)
            plots.append(Gnuplot.File(fn, with_='lines linecolor %d' % color, title='%s (%.2f - %.2f)' % (plot.title, plot.min, plot.max)))
            if plot.IsMidLineVisible():
                plots.append(Gnuplot.Func('%.4f' % plot.GetMidLine(), with_='lines linecolor %d' % color, title=''))
            color += 1
        g.plot(*plots)
        wait()
