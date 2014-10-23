from woanalyzer import *

class WorkOutProcessor:
    def __init__(self, activities, programOptions):
        self.activities = activities
        self.programOptions = programOptions
    
    def DumpProgressCsv(self):
        print 'date, time, Watts per bpm, avg power (watts), avg HR, avg speed (m/s), work (joules), time (seconds), distance (meters)'
        for activity in self.activities:
            wa = self.AnalyzeActivity(activity)
            if not wa:
                print '-, -, -, -, -, -, -, -, -'
            else:
                parts = activity.id.split('T')
                d = parts[0]
                t = parts[1][:-1]
                print '%s, %s, %s, %s, %s, %s, %s, %s, %s' % (d, t, wa.avgPedalingPower / wa.avgHr, wa.avgPedalingPower, wa.avgHr, wa.AverageSpeed(), wa.joules, wa.TotalTime(), wa.totalDistance)
    
    def DumpActivities(self):
        for activity in self.activities:
            self.DumpActivity(activity)
    
    def AnalyzeActivity(self, activity):
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
            return wa
        return None
        
    def DumpActivity(self, activity):
        print 'Activity %s' % activity.id
        print '============================='
        wa = self.AnalyzeActivity(activity)
        if wa:
            if self.programOptions.DoPlots():
                wa.Plot(self.programOptions)
            self.DumpAnalyzerData(wa)
        print '          Total activity time = %.2f' % activity.totalTime

    def DumpActivitiesByLaps(self):
        for activity in self.activities:
            self.DumpActivity(activity)
            print ''
            if self.programOptions.DumpLaps():
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
            if self.programOptions.DoPlots():
                wa.Plot(self.programOptions)
            self.DumpAnalyzerData(wa, 5)
            print '      Analyzer joules / Reported Calories = %f kCal' % (wa.joules / lap.reportedCalories if lap.reportedCalories > 0 else 0)
            print '                        Reported Calories = %f kCal' % lap.reportedCalories

    def DumpAnalyzerData(self, wa, spacing=1):
            print '%s               average speed = %.1f m/s' % (' ' * spacing, wa.AverageSpeed())
            print '%s   power / hr while pedaling = %.2f W/bpm' % (' ' * spacing, wa.avgPedalingPower / wa.avgHr)
            print '%sAverage power while pedaling = %.1f W' % (' ' * spacing, wa.avgPedalingPower)
            print '%s   Average hr while pedaling = %.f bpm' % (' ' * spacing, wa.avgHr)
            print '%s                 My Calories = %f kCal' % (' ' * spacing, wa.joules / 640)
        