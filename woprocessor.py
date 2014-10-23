from woanalyzer import *

class WorkOutProcessor:
    def __init__(self, activities):
        self.activities = activities
    
    def DumpActivities(self):
        for activity in self.activities:
            self.DumpActivity(activity)
    
    def DumpActivity(self, activity):
        print 'Activity %s' % activity.id
        print '============================='
        times = []
        distances = []
        altitudes = []
        heartRates = []
        reportedCalories = 0
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
            self.DumpAnalyzerData(wa)
        print     '          Total activity time = %.2f' % activity.totalTime

    def DumpActivitiesByLaps(self):
        for activity in self.activities:
            self.DumpActivity(activity)
            print ''
            self.DumpLaps(activity)
            
    def DumpLaps(self, activity):
        for lap in activity.laps:
            print '    Lap %s' % lap.name
            print '    ------------------------'
            self.DumpLap(lap)
            
    def DumpLap(self, lap):
        staticData = StaticData(88.0, COEFF_FRIC_MTB, 0.5)

        if len(lap.times) > 2:
            wa = WorkOutAnalyzer(staticData, lap.times, lap.distances, lap.altitudes, lap.heartRates)
            wa.Analyze()
            wa.Plot()
            self.DumpAnalyzerData(wa, 5)
            print '      Analyzer joules / Reported Calories = %f' % (wa.joules / lap.reportedCalories if lap.reportedCalories > 0 else 0)
            print '                        Reported Calories = %f' % lap.reportedCalories

    def DumpAnalyzerData(self, wa, spacing=1):
            print '%s   power / hr while pedaling = %.2f' % (' ' * spacing, wa.avgPedalingPower / wa.avgHr)
            print '%sAverage power while pedaling = %.1f' % (' ' * spacing, wa.avgPedalingPower)
            print '%s   Average hr while pedaling = %.f bpm' % (' ' * spacing, wa.avgHr)
            print '%s                 My Calories = %f' % (' ' * spacing, wa.joules / 640)
        