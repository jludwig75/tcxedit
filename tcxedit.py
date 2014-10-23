import sys
import os

from tcxparser import *
from woanalyzer import *


class WorkOutProcessor:
    def __init__(self, activities):
        self.activities = activities
    
    def Dump(self):
        for activity in self.activities:
            times = []
            distances = []
            altitudes = []
            heartRates = []
            reportedCalories = 0
            print 'Activity %s' % activity.id
            #print activity.id,
            totalActivityTime = 0.0
            lastTime = 0
            for lap in activity.laps:
                reportedCalories += lap.reportedCalories
                tempTimes = [t + lastTime for t in lap.times]
                times += tempTimes
                distances += lap.distances
                altitudes += lap.altitudes
                heartRates += lap.heartRates
                lastTime = times[-1]
            staticData = StaticData(88.0, COEFF_FRIC_MTB, 0.5)
    
            if len(times) > 2:
                wa = WorkOutAnalyzer(staticData, times, distances, altitudes, heartRates)
                wa.Analyze()
                wa.Plot()
                #print ', %.3f' % (wa.avgPedalingPower / wa.avgHr)
                print '  Average power while pedaling = %.1f' % wa.avgPedalingPower
                print '     Average hr while pedaling = %.f bpm' % wa.avgHr
                print '     power / hr while pedaling = %.2f' % (wa.avgPedalingPower / wa.avgHr)
                print '    Analyzer joules / Calories = %f' % (wa.joules / lap.reportedCalories if lap.reportedCalories > 0 else 0)
                print '                      Calories = %f' % lap.reportedCalories
                print '                   My Calories = %f' % (wa.joules / 640)
            print     '           Total activity time = %.2f' % activity.totalTime
    
    def DumpLap(self, lap):
        staticData = StaticData(88.0, COEFF_FRIC_MTB, 0.5)

        if len(lap.times) > 2:
            wa = WorkOutAnalyzer(staticData, lap.times, lap.distances, lap.altitudes, lap.heartRates)
            wa.Analyze()
            print '  Average power while pedaling = %.1f' % wa.avgPedalingPower
            print '     Average hr while pedaling = %.f bpm' % wa.avgHr
            print '     power / hr while pedaling = %.2f' % (wa.avgPedalingPower / wa.avgHr)
            print '    Analyzer joules / Calories = %f' % (wa.joules / lap.reportedCalories if lap.reportedCalories > 0 else 0)
            print '                      Calories = %f' % lap.reportedCalories
            print '                   My Calories = %f' % (wa.joules / 640)

            
def main(programName, args):
    TestForceCaclulations()
    fileName = args[0]
    
    stopTime = 0
    if len(args) > 1:
        stopTime = int(args[1])
    
    print 'Processing file "%s"' % fileName
    
    tcxFile = TcxFile.ParseFile(fileName, stopTime)
    
    wpa = WorkOutProcessor(tcxFile.activities)
    wpa.Dump()
    
    if stopTime != 0:
        tcxFile.write('new.tcx')
    
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv[0], sys.argv[1:])) 