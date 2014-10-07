import sys
import xml.etree.ElementTree as ET
import time
import math
import Gnuplot
import tempfile
import os
import numpy

def wait(str=None, prompt='Press return to show results...\n'):
    if str is not None:
        print str
    raw_input(prompt)



TIME_FORMAT_STRING = '%Y-%m-%dT%H:%M:%SZ'
NS = '{http://www.garmin.com/xmlschemas/TrainingCenterDatabase/v2}'

def TAG_NAME(name):
    return NS + name

ACCEL_OF_GRAVITY = 9.8
AIR_DENSITY = 1.226
DRAG_COEFFICIENT = 0.5
COEFF_FRIC_MTB = 0.015
COEFF_FRIC_ROAD = 0.06

class StaticData:
    def __init__(self, weight, friction, frontalArea):
        self.weight = weight
        self.friction = friction
        self.frontalArea = frontalArea

def ForceOfDrag(staticData, airSpeed):
    return staticData.frontalArea * AIR_DENSITY * airSpeed * airSpeed * DRAG_COEFFICIENT / 2

def CalcForceOfMotion(staticData, hDistance, vDistance, time):
    if hDistance == 0:
        return 0
    absDistance = math.sqrt(hDistance * hDistance + vDistance * vDistance)
    airSpeed = absDistance / time
    gravitatyForce = staticData.weight * ACCEL_OF_GRAVITY * vDistance / hDistance
    #print 'Slope force = %.1f Nm' % gravitatyForce
    frictionForce = staticData.friction * staticData.weight * ACCEL_OF_GRAVITY# * hDistance / vDistance
    #print 'Rolling resistance = %.1f Nm' % frictionForce 
    dragForce = ForceOfDrag(staticData, airSpeed)
    #print 'Drag resistance = %.1f Nm' % dragForce
    totalForce = gravitatyForce + frictionForce + dragForce
    work = totalForce * absDistance
    power = totalForce * airSpeed
    #print '%.3f Nm, %.3f Joules %f kcal, %.2f Watts, %.3f joules' % (totalForce, work, JoulesToKCalories(work), power, power * time) 
    return totalForce
    

def CalcWork(staticData, hDistance, vDistance, time):
    absDistance = math.sqrt(hDistance * hDistance + vDistance * vDistance)
    return CalcForceOfMotion(staticData, hDistance, vDistance, time) * absDistance

def JoulesToKCalories(joules):
    return joules * 0.000239005736

def TestForceCaclulations():
    sd = StaticData(88, 0.004, 0.5)
    force = CalcForceOfMotion(sd, 100.0, 3.0, 12.5)
    joules = CalcWork(sd, 100.0, 3.0, 12.5)
    kcal = JoulesToKCalories(joules)
    print 'Force = %.2f Nm avg, %.2f Watts avg, %.2f joules, %.2f kcal' % (force, force * 8, joules, kcal)


class TackPointDataMissing(Exception):
    def __init__(self, message):
        super(Exception, self).__init__(message)

def GreatCircleDistance(pos1, pos2):
    phi1 = math.radians(pos1.latitude)
    phi2 = math.radians(pos2.latitude)
    deltaPhi = math.radians(pos2.latitude - pos1.latitude)
    deltaLambda = math.radians(pos2.longitude - pos1.longitude)
    a = math.sin(deltaPhi/2) * math.sin(deltaPhi/2) + math.cos(phi1) * math.cos(phi2) * math.sin(deltaLambda / 2) * math.sin(deltaLambda / 2)
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    return 6371000 * c
    
class Position:
    def __init__(self, positionXml):
        self.latitude = float(positionXml.find(TAG_NAME('LatitudeDegrees')).text)
        self.longitude = float(positionXml.find(TAG_NAME('LongitudeDegrees')).text)
    
    def Distance(self, other):
        return GreatCircleDistance(self, other)
        
class TrackPointInfo:
    def __init__(self, trackPointXml):
        timeStr = trackPointXml.find(TAG_NAME('Time')).text
        t = time.strptime(timeStr, TIME_FORMAT_STRING)
        self.time = time.mktime(t)
        
        altNode = trackPointXml.find(TAG_NAME('AltitudeMeters'))
        if altNode is None:
            raise TackPointDataMissing("no altitude")
        self.altitude = float(altNode.text)
        
        distNode = trackPointXml.find(TAG_NAME('DistanceMeters'))
        if distNode is None:
            raise TackPointDataMissing("no distance")
        self.distance = float(distNode.text)
        
        hrNode = trackPointXml.find(TAG_NAME('HeartRateBpm'))
        if hrNode is not None:
            self.heartRate = int(hrNode.find(TAG_NAME('Value')).text)
        else:
            self.heartRate = 0

        posNode = trackPointXml.find(TAG_NAME('Position'))
        if posNode is not None:
            self.position = Position(posNode)
        else:
            self.position = None

        #print '    "tp" : {"t" : %.1f, "alt" : %.3f, "dis" : %.3f,' % (self.time, self.altitude, self.distance)
        #if self.position:
        #    print '            "hr" : %d, "pos" : {"lat" : %.4f, "lon" : %.4f} }' % (self.heartRate, self.position.latitude, self.position.longitude)

def ErrorPercent(actual, calculated):
    if actual == 0:
        return 0
    return 100.0 * (float(calculated) - float(actual)) / float(actual)

def ApplyFilter(data, filter):
    mid = len(filter) / 2
    new_data = [0] * len(data)
    for i in range(len(data)):
        sum = 0
        count = 0
        for j in range(-mid, mid + 1):
            idx = i + j
            if idx >= 0 and idx < len(data):
                sum += data[idx] * filter[mid + j]
                count += 1
        new_data[i] = sum / count
    return new_data
            
def SmoothData(dataArray, width):
    filter = [1] * width
    return ApplyFilter(dataArray, filter)


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
        g = Gnuplot.Gnuplot(debug=1)
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
        wait('Press Enter')


class WorkOutAnalyzer:
    def __init__(self, staticData, times, distances, altitudes, heartRates):
        self.staticData = staticData
        self.times = times
        self.distances = distances
        self.altitudes = altitudes
        self.heartRates = heartRates
    
    def Analyze(self):
        self.PreProcess()
        self.Process()
        self.PostProcess()
    
    def PreProcess(self):
        # Smooth sensor data
        self.altitudes = SmoothData(self.altitudes, 3)
        self.heartRates = SmoothData(self.heartRates, 3)
        self.distances = SmoothData(self.distances, 3)
        
        # Calculate speeds
        self.speeds = [0] * len(self.times)
        self.slopes = [0] * len(self.times)
        self.vDistances = [0] * len(self.times)
        lastTime = 0
        lastDistance = 0
        lastAltitude = self.altitudes[0]
        for i in range(len(self.times)):
            distance = self.distances[i]
            vDistance = self.altitudes[i] - lastAltitude
            self.speeds[i] = distance / (self.times[i] - lastTime)

            # Handle slope here
            
            # Ignore negligible values 
            if distance <= 0.1:
                distance = 0
            if abs(vDistance) < 0.1:
                vDistance = 0
                
            # Don't allow a rise when there was no horizontal distance traveled
            if distance == 0:
                vDistance = 0
            elif abs(vDistance) > 0.5 * distance:    # Don't allow more then 50% grade
                vDistance = 0.5 * distance * vDistance / abs(vDistance)
                
            # Calculate the horizontal distance
            if distance == 0:
                hDistance = 0
            else:
                hDistance = math.sqrt(distance * distance - vDistance * vDistance)
                
            self.distances[i] = distance
            self.vDistances[i] = vDistance
            
            # Now calculate the slope
            if hDistance == 0:    
                self.slopes[i] = 0  # To avoid divide by zerp
            else:
                slope = math.atan(vDistance / hDistance)
                if slope > 0.2 or slope < -0.15:
                    print '%.4f / %.4f' % (vDistance, hDistance)
                self.slopes[i] = slope

            lastTime = self.times[i]
            lastDistance = self.distances[i]
            lastAltitude = self.altitudes[i]
        
        # Smooth the speeds
        self.speeds = SmoothData(self.speeds, 3)
        # Smooth the slopes
        self.slopes = SmoothData(self.slopes, 3)
    
    def CalculateExternalForces(self):
        self.ft = [0] * len(self.times)
        self.ff = [0] * len(self.times)
        self.fd = [0] * len(self.times)
        self.fs = [0] * len(self.times)
        for i in range(1, len(self.times)):
            speedIn = self.speeds[i-1]
            speedOut = self.speeds[i]
            intervalTime = self.times[i] - self.times[i-1]
            avgSpeed = (speedIn + speedOut) / 2 
            vDistance = self.vDistances[i]
            distance = self.distances[i]
            hDistance = math.sqrt(distance * distance - vDistance * vDistance)
            #print 'sIn = %.2f, sOut = %.2f, sAvg = %.2f, iTime = %.2f, dist = %.3f, vDist = %.3f, hDist = %.3f' % (speedIn, speedOut, avgSpeed, intervalTime, distance, vDistance, hDistance)
            # Calculate total force Ft from change in momentum
            Ft = self.staticData.weight * (speedOut - speedIn) / intervalTime
            self.ft[i] = Ft
            # Calculate component forces
            Ff = self.staticData.friction * self.staticData.weight * ACCEL_OF_GRAVITY
            self.ff[i] = Ff
            Fd = ForceOfDrag(self.staticData, avgSpeed)
            self.fd[i] = Fd
            if distance > 0:
                Fs = self.staticData.weight * ACCEL_OF_GRAVITY * vDistance / distance
            else:
                Fs = 0
            self.fs[i] = Fs

    def CalculateRiderForce(self):
        self.fr = [0] * len(self.times)
        for i in range(1, len(self.times)):
            self.fr[i] = self.ft[i] + (self.ff[i] + self.fd[i] + self.fs[i])
        
    def CalculateWorkAndPower(self):
        self.power = [0] * len(self.times)
        for i in range(1, len(self.times)):
            self.power[i] = self.fr[i] * self.speeds[i]
        
    def Process(self):
        self.CalculateExternalForces()
        
        # Smooth the forces
        #self.fd = SmoothData(self.fd, 3)
        #self.ft = SmoothData(self.ft, 3)
        #self.fs = SmoothData(self.fs, 3)
        #self.ff = SmoothData(self.ff, 3)
        
        #self.fr = SmoothData(self.fr, 3)
        
        self.CalculateRiderForce()
        
        self.CalculateWorkAndPower()
    
    def PostProcess(self):
        # Smooth the power
        self.power = SmoothData(self.power, 5)
        
        mp = MuliPlotter(10000)
        
        mp.AddPlot(PlotData("speed", self.times, self.speeds))
        mp.AddPlot(PlotData("altitude", self.times, self.altitudes))
        #mp.AddPlot(PlotData("distance", self.times, self.distances))
        #mp.AddPlot(PlotData("total force", self.times, self.ft))
        #mp.AddPlot(PlotData("slope", self.times, self.slopes))
        #mp.AddPlot(PlotData("slope force", self.times, self.fs))
        #mp.AddPlot(PlotData("drag", self.times, self.fd))
        #mp.AddPlot(PlotData("Fr", self.times, self.ft))
        #mp.AddPlot(PlotData("Ft", self.times, self.fr))
        mp.AddPlot(PlotData("power", self.times, self.power))
        
        mp.Plot()
        sys.exit(0)
        
    
class LapInfo:
    def __init__(self, lapXml):
        
        self.lapXml = lapXml
        
        self.reportedTotalTime = float(lapXml.find(TAG_NAME('TotalTimeSeconds')).text)
        self.reportedDistance = float(lapXml.find(TAG_NAME('DistanceMeters')).text)
        self.reportedMaxSpeed = float(lapXml.find(TAG_NAME('MaximumSpeed')).text)
        self.reportedCalories = int(lapXml.find(TAG_NAME('Calories')).text)
        avgHrNode = lapXml.find(TAG_NAME('AverageHeartRateBpm'))
        if avgHrNode is not None:
            self.reportedAvgHearRate = int(avgHrNode.find(TAG_NAME('Value')).text)
        else:
            self.reportedAvgHearRate = 0
        maxHrNode = lapXml.find(TAG_NAME('MaximumHeartRateBpm'))
        if maxHrNode is not None:
            self.reportedMaxHearRate = int(maxHrNode.find(TAG_NAME('Value')).text)
        else:
            self.reportedMaxHearRate = 0
        
        self.lastTime = 0
        self.startTime = 0
        self.timeElapsed = 0
        self.totalDistance = 0
        self.maxSpeed = 0
        self.avgHeartRate = 0
        self.maxHeartRate = 0
        self.dataPoints = 0
        self.calories = 0
        self.lastDistance = 0
        self.lastPositiion = None
        self.calculatedDistance = 0
        self.gpsDistance = 0
        self.lastPositionAnywhere = None
        self.lastAltitude = 0
        self.metersAscended = 0
        self.metersDescended = 0
        self.timeAscending = 0
        self.timeDescending = 0
        self.ascendingMeters = 0
        self.descendingMeters = 0
        self.joules = 0
        
        self.times = []
        self.altitudes = []
        self.heartRates = []
        self.distances = []
        self.speeds = []
        self.powers = []
    
    def UpdateLapStats(self):
        self.lapXml.find(TAG_NAME('TotalTimeSeconds')).text = '%.7f' % self.GetTotalRecordedTime()
        self.lapXml.find(TAG_NAME('DistanceMeters')).text = '%.7f' % self.lastDistance
        self.lapXml.find(TAG_NAME('MaximumSpeed')).text = '%.7f' % self.maxSpeed
        avgHrNode = self.lapXml.find(TAG_NAME('AverageHeartRateBpm'))
        if avgHrNode is not None:
            avgHrNode.find(TAG_NAME('Value')).text = str(int(round(self.avgHeartRate)))
        maxHrNode = self.lapXml.find(TAG_NAME('MaximumHeartRateBpm'))
        if maxHrNode is not None:
            maxHrNode.find(TAG_NAME('Value')).text = str(self.maxHeartRate)
        extensions = self.lapXml.find(TAG_NAME('Extensions'))
        if extensions is not None:
            lx = extensions.find('{http://www.garmin.com/xmlschemas/ActivityExtension/v2}LX')
            if lx is not None:
                avgSpeed = lx.find('{http://www.garmin.com/xmlschemas/ActivityExtension/v2}AvgSpeed')
                if avgSpeed is not None:
                    avgSpeed.text = '%.7f' % (self.lastDistance / self.GetTotalRecordedTime())
    
    def GetTotalRecordedTime(self):
        return self.lastTime - self.startTime
    
    def Dump(self):
        staticData = StaticData(88.0, COEFF_FRIC_MTB, 0.5)
        print '    Reported Data:'
        print '      Total time = %f' % self.reportedTotalTime
        print '      Total distance = %f' % self.reportedDistance
        print '      Max Speed = %f %f mph' % (self.reportedMaxSpeed, 2.23694 * self.reportedMaxSpeed)
        print '      Avg HR = %d' % self.reportedAvgHearRate
        print '      Max HR = %d' % self.reportedMaxHearRate
        print '      Calories = %.f' % self.reportedCalories
        print '    Calculated Data:'
        print '      Total time = %f' % (self.lastTime - self.startTime)
        print '      Total distance = %f' % self.lastDistance
        print '      Max Speed = %f %f mph (%.2f%% error)' % (self.maxSpeed, 2.23694 * self.maxSpeed, ErrorPercent(self.reportedMaxSpeed, self.maxSpeed))
        print '      Avg HR = %d (%.2f%% error)' % (int(round(self.avgHeartRate)), ErrorPercent(round(self.reportedAvgHearRate), round(self.avgHeartRate)))
        print '      Max HR = %d' % self.maxHeartRate
        print '      Joules = %.1f' % self.joules
        print '      Joules / Calories = %f' % (self.joules / self.reportedCalories)
        joulesAscending = CalcWork(staticData, self.ascendingMeters, self.metersAscended, self.timeAscending)
        joulesDescending = CalcWork(staticData, self.descendingMeters, -self.metersDescended, self.timeDescending)
        totalJoules = joulesAscending
        if  joulesDescending > 0:
            totalJoules += joulesDescending
        print '      Meters ascended  = %.1f in %.1f meters %.1f seconds = %.1f joules' % (self.metersAscended, self.ascendingMeters, self.timeAscending, joulesAscending) 
        print '      Meters descended = %.1f in %.1f meters %.1f seconds = %.1f joules' % (self.metersDescended, self.descendingMeters, self.timeDescending, joulesDescending)
        print '      Total joules = %f, joules / calories = %f' % (totalJoules, totalJoules / self.reportedCalories)
        wa = WorkOutAnalyzer(staticData, self.times, self.distances, self.altitudes, self.heartRates)
        wa.Analyze()
        #sys.exit(0)
    
    def Plot(self):
        
        self.powers = SmoothData(self.powers, 15)
        self.speeds = SmoothData(self.speeds, 15)
        
        g = Gnuplot.Gnuplot(debug=1)
        g.clear()
        filename1 = tempfile.mktemp()
        f = open(filename1, 'w')
        filename2 = tempfile.mktemp()
        f2 = open(filename2, 'w')
        filename3 = tempfile.mktemp()
        f3 = open(filename3, 'w')
        filename4 = tempfile.mktemp()
        f4 = open(filename4, 'w')
        try:
            for i in range(len(self.times)):
                f.write('%s %s\n' % (self.times[i], self.altitudes[i]))
            f.close()

            for i in range(len(self.times)):
                f2.write('%s %s\n' % (self.times[i], self.speeds[i]))
            f2.close()

            for i in range(len(self.times)):
                f3.write('%s %s\n' % (self.times[i], self.heartRates[i]))
            f3.close()

            for i in range(len(self.times)):
                f4.write('%s %s\n' % (self.times[i], self.powers[i]))
            f4.close()

            g.xlabel('time (seconds)')
            
            #g("set yrange [1670:1800]")
            #g("set ytics 100 nomirror tc lt 1")
            #g("set ylabel 'altitude' tc lt 1")

            #g("set y2range [0:10]")
            #g("set y2tics 5 nomirror tc lt 2")
            #g("set y2label 'speed' tc lt 2")
            
            g('set multiplot')
                        
            g.plot(Gnuplot.File(filename1, with_='lines linecolor 1'))
            g.plot(Gnuplot.File(filename2, with_='lines linecolor 2'))
            g("set yrange [0:200]")
            g.plot(Gnuplot.File(filename3, with_='lines linecolor 3'))
            g("set yrange [0:300]")
            g.plot(Gnuplot.File(filename4, with_='lines linecolor 4'))
            
            g('unset multiplot')
            #g.plot(Gnuplot.File(filename1, with_='linetype 1'),
            #       Gnuplot.File(filename2, with_='linetype 2'))
            wait('Set title and axis labels and try replot()')
    
        finally:
            os.unlink(filename1)
            os.unlink(filename2)
            os.unlink(filename3)
            os.unlink(filename4)
            
        
    def ProcessTrackPoint(self, trackPoint, activityStartTime, stopTime):
        activityTimeElapsed = trackPoint.time - activityStartTime
        if stopTime != 0 and activityTimeElapsed > stopTime:
            self.UpdateLapStats()
            return False
        if 0 == self.startTime:
            self.startTime = trackPoint.time
            self.avgHeartRate = trackPoint.heartRate
        else:
            elapsed = trackPoint.time - self.startTime
            intervalTime = trackPoint.time - self.lastTime
            intervalDistance = trackPoint.distance - self.lastDistance
            vDistance = trackPoint.altitude - self.lastAltitude
            if intervalDistance >= abs(vDistance):
                hDistance = math.sqrt(intervalDistance * intervalDistance - vDistance * vDistance)
            else:
                hDistance = intervalDistance
            
            if vDistance >= 0:
                self.ascendingMeters += intervalDistance
                self.metersAscended += vDistance
                self.timeAscending += intervalTime
            else:
                self.descendingMeters += intervalDistance
                self.metersDescended -= vDistance
                self.timeDescending += intervalTime
                
            staticData = StaticData(88.0, COEFF_FRIC_MTB, 0.5)
            work = CalcWork(staticData, hDistance, vDistance, intervalTime)
            if work > 0:
                self.joules += work
            
            speed = intervalDistance / intervalTime
            if speed > self.maxSpeed:
                self.maxSpeed = speed

            power = CalcForceOfMotion(staticData, hDistance, vDistance, intervalTime) * speed
            self.times.append(elapsed)
            self.heartRates.append(trackPoint.heartRate)
            self.altitudes.append(trackPoint.altitude)
            self.distances.append(intervalDistance)
            self.speeds.append(speed)
            self.powers.append(power if power > 0 else 0)
            
            if trackPoint.position and self.lastPositiion:
                gpsIntervalDistance = trackPoint.position.Distance(self.lastPositiion)
                gpsSpeed = gpsIntervalDistance / intervalTime
                self.calculatedDistance += gpsIntervalDistance 
            else:
                gpsIntervalDistance = None
                gpsSpeed = None
                self.calculatedDistance += intervalDistance

            if trackPoint.position and self.lastPositionAnywhere:
                self.gpsDistance += trackPoint.position.Distance(self.lastPositionAnywhere)
            
            # Heart rate
            if trackPoint.heartRate > self.maxHeartRate:
                self.maxHeartRate = trackPoint.heartRate
            
            self.avgHeartRate = (self.avgHeartRate * (elapsed - intervalTime) + trackPoint.heartRate * intervalTime) / elapsed
            
            #print '      elapsed time = %f' % elapsed
            #print '      interval time = %f' % intervalTime
            #print '      interval distance = %f' % intervalDistance
            #if gpsIntervalDistance is not None:
            #    print '      GPS interval distance = %f, %.3f%% error' % (gpsIntervalDistance, ErrorPercent(intervalDistance, gpsIntervalDistance))
            #    print '      GPS speed = %f' % (2.23694 * gpsSpeed)
            #print '      avg HR = %f' % self.avgHeartRate
            #print '      Speed = %f' % (2.23694 * speed)
        
        self.lastTime = trackPoint.time
        self.lastDistance = trackPoint.distance
        self.lastPositiion = trackPoint.position
        self.lastAltitude = trackPoint.altitude
        if trackPoint.position:
            self.lastPositionAnywhere = trackPoint.position
        
        return True
    

class Activity:
    def __init__(self):
        self.laps = []
        self.totalTime = 0
    
    def AddLap(self, lap):
        self.laps.append(lap)
        self.totalTime += lap.lastTime - lap.startTime

class TcxVisitor:
    def __init__(self, stopTime):
        self.stopTime = stopTime
        self.activities = []
        self.currentActivity = None
        self.currentLap = None
        self.lastActivityStartTime = 0
    
    def EndLap(self):
        self.currentActivity.AddLap(self.currentLap)
        self.currentLap = None
    
    def EndActivity(self):
        self.activities.append(self.currentActivity)
        self.currentActivity = None
    
    def VisitActivity(self, activityXml):
        self.lastActivityStartTime = 0
        self.currentActivity = Activity()
    
    def VisitLap(self, lapXml):
        self.currentLap = LapInfo(lapXml)
    
    def VisitTackPoint(self, trackPointXml):
        try:
            trackPoint = TrackPointInfo(trackPointXml)
            if self.lastActivityStartTime == 0:
                self.lastActivityStartTime = trackPoint.time
            return self.currentLap.ProcessTrackPoint(trackPoint, self.lastActivityStartTime, self.stopTime)
        except TackPointDataMissing as e:
            return True
            
    def Dump(self):
        for activity in self.activities:
            print 'Activity'
            totalActivityTime = 0.0
            for lap in activity.laps:
                print '  Lap'
                lap.Dump()
                #lap.Plot()
            print ' Total activity time = %.2f' % activity.totalTime


def VisitTack(root, visitor):
    keepData = True
    activities = root.find(TAG_NAME('Activities'))
    if activities is None:
        return
    for activity in activities.findall(TAG_NAME('Activity')):
        if keepData:
            visitor.VisitActivity(activity)
        else:
            root.remove(activity)
        for lap in activity.findall(TAG_NAME('Lap')):
            if keepData:
                visitor.VisitLap(lap)
            else:
                activity.remove(lap)
                continue
            for track in lap.findall(TAG_NAME('Track')):
                for trackPoint in track.findall(TAG_NAME('Trackpoint')):
                    if keepData:
                        if not visitor.VisitTackPoint(trackPoint):
                            keepData = False
                            parent = lap.find(TAG_NAME('Track'))
                            track.remove(trackPoint)
                    else:
                        track.remove(trackPoint)
            visitor.EndLap()
        visitor.EndActivity()

def main(programName, args):
    TestForceCaclulations()
    fileName = args[0]
    
    stopTime = 0
    if len(args) > 1:
        stopTime = int(args[1])
    
    print 'Processing file "%s"' % fileName
    
    tree = ET.parse(fileName)
    
    ET.register_namespace('', 'http://www.garmin.com/xmlschemas/ActivityExtension/v2')
    ET.register_namespace('', 'http://www.garmin.com/xmlschemas/TrainingCenterDatabase/v2')
    
    root = tree.getroot()
    
    visitor = TcxVisitor(stopTime)

    VisitTack(root, visitor)
    
    visitor.Dump()
    
    tree.write('new.tcx', xml_declaration=True,encoding='utf-8', method="xml")
    
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv[0], sys.argv[1:])) 