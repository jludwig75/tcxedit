class ProgramOptions:
    def __init__(self, fileName, plot, dumpLaps):
        self._fileName = fileName
        self._plot = plot
        self._dumpLaps = dumpLaps
    
    @staticmethod
    def Help():
        print 'tcxedit.py [options] <tcx_file_name>'
        print '  tcx_file_name - The TCX file to analyze'
        print '  Options:'
        print '    -p=<plot_options>  Speficy graphs to plot'
        print '    -l                 Display individual laps'
        print '  plot options'
        print '    a   Altitude'
        print '    s   Speed'
        print '    h   Heart rate'
        print '    p   Pedaling power'
        print '    r   Power to heart rate ratio while pedaling'
        print '    w   Power (Includes braking power)'
        print '    j   Plot cummulative work done in joules'
        print '    d   Plot distances'
        print '    f   Plot forces'
        print '    g   Plot slope grades'
        print '    t   Shows corresponding trend lines if available for the select plots'
    
    @staticmethod
    def ParseCommandLine(args):
        plotOptions = ''
        dumpLaps = False
        fileName = None
        for arg in args:
            if arg[0] == '-':
                if len(arg) < 2:
                    raise Exception('Command line parse error: invalid command line option "%s"' % arg)
                if arg[1] == 'l':
                    dumpLaps = True
                elif arg[1] == 'p':
                    if len(arg) < 4 or arg[2] != '=':
                        raise Exception('Command line parse error: invalid plot option "%s"' % arg)
                    plotOptions = arg[3:]
                elif arg[1] == 'h' or arg[1] == '?':
                    raise Exception("Command usage:")
                else:
                    raise Exception('Command line parse error: unknown command line option "%s"' % arg)
            else:
                if fileName != None:
                    raise Exception('Command line parse error: More than one file name specified')
                fileName = arg
        if fileName is None:
            raise Exception('Command line parse error: No file name specified')
        return ProgramOptions(fileName, plotOptions, dumpLaps)
    
    def FileName(self):
        return self._fileName
    
    def DumpLaps(self):
        return self._dumpLaps
    
    # Plot options
    def DoPlots(self):
        return len(self._plot) > 0
    
    def DoPedalingPowerPlot(self):
        return 'p' in self._plot
    
    def DoAltitudePlot(self):
        return 'a' in self._plot
    
    def DoHeartRatePlot(self):
        return 'h' in self._plot
    
    def DoSpeedPlot(self):
        return 's' in self._plot
    
    def DoGradePlot(self):
        return 'g' in self._plot
    
    def DoTrendPlots(self):
        return 't' in self._plot
    
    def DoPowerPlot(self):
        return 'w' in self._plot

    def DoPwrToHrPlot(self):
        return 'r' in self._plot
    
    def DoForcePlots(self):
        return 'f' in self._plot

    def DoDistancePlots(self):
        return 'd' in self._plot

    def DoJoulesPlot(self):
        return 'j' in self._plot
        