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

GAUSSIAN_SMOOT_SDEV = 2

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
            self.heartRate = float(hrNode.find(TAG_NAME('Value')).text)
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
    mid = width / 2
    filter = [0] * width
    for x in range(-mid, mid + 1):
        filter[mid + x] = 1 / math.sqrt(2 * math.pi * sdev) * math.exp(-(x * x) / (2 * sdev * sdev))
    return filter
                
def SmoothData(dataArray, sdev):
    filter = BuildGaussianFilter(2 * ((sdev * 10) / 2) + 1, sdev)#[1] * width
    filter_sum = 0
    for value in filter:
        filter_sum += value
    print 'Filter = %s = %s' % (filter, filter_sum)
    return ApplyFilter(dataArray, filter, filter_sum)


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
        if len(self.times) < 2 or len(self.altitudes) < len(self.times):
            return
        self.altitudes = SmoothData(self.altitudes, GAUSSIAN_SMOOT_SDEV)
        self.heartRates = SmoothData(self.heartRates, GAUSSIAN_SMOOT_SDEV)
        self.distances = SmoothData(self.distances, GAUSSIAN_SMOOT_SDEV)
        
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
                self.slopes[i] = slope

            lastTime = self.times[i]
            lastDistance = self.distances[i]
            lastAltitude = self.altitudes[i]
        
        # Smooth the speeds
        self.speeds = SmoothData(self.speeds, GAUSSIAN_SMOOT_SDEV)
        # Smooth the slopes
        self.slopes = SmoothData(self.slopes, GAUSSIAN_SMOOT_SDEV)
    
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
            self.ft[i] = self.staticData.weight * (speedOut - speedIn) / intervalTime

            # Calculate component forces
            # Friction
            self.ff[i] = self.staticData.friction * self.staticData.weight * ACCEL_OF_GRAVITY
            # Drag
            self.fd[i] = ForceOfDrag(self.staticData, avgSpeed)
            
            # Slope force (due to gravity)
            if distance > 0:
                self.fs[i] = self.staticData.weight * ACCEL_OF_GRAVITY * vDistance / distance
            else:
                self.fs[i] = 0

    def CalculateRiderForce(self):
        self.fr = [0] * len(self.times)
        for i in range(1, len(self.times)):
            self.fr[i] = self.ft[i] + (self.ff[i] + self.fd[i] + self.fs[i])
        
    def CalculateWorkAndPower(self):
        self.power = [0] * len(self.times)
        self.joules = 0
        self.joulesPerInterval = [0] * len(self.times)
        self.joulesCum = [0] * len(self.times)
        for i in range(1, len(self.times)):
            self.power[i] = self.fr[i] * self.speeds[i]
            if self.fr[i] < 0:
                joules = 0
            else:
                joules = self.fr[i] * self.distances[i]
            self.joules += joules
            self.joulesPerInterval[i] = joules
            self.joulesCum[i] = self.joules
        
    def Process(self):
        self.CalculateExternalForces()
        
        # Smooth the forces
        #self.fd = SmoothData(self.fd, GAUSSIAN_SMOOT_SDEV)
        #self.ft = SmoothData(self.ft, GAUSSIAN_SMOOT_SDEV)
        #self.fs = SmoothData(self.fs, GAUSSIAN_SMOOT_SDEV)
        #self.ff = SmoothData(self.ff, GAUSSIAN_SMOOT_SDEV)
        
        #self.fr = SmoothData(self.fr, GAUSSIAN_SMOOT_SDEV)
        
        self.CalculateRiderForce()
        
        self.CalculateWorkAndPower()
    
    def PostProcess(self):
        # Smooth the power
        self.power = SmoothData(self.power, GAUSSIAN_SMOOT_SDEV)
        
        #self.CalcHRPowerFactor(30)
        
        #self.FindTimeSkew()
        
        #self.power = SmoothData(self.power, GAUSSIAN_SMOOT_SDEV)
        #self.heartRates = SmoothData(self.heartRates, GAUSSIAN_SMOOT_SDEV)
        #self.speeds = SmoothData(self.speeds, GAUSSIAN_SMOOT_SDEV)
        
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
        #mp.AddPlot(PlotData("work", self.times, self.joulesCum))
        mp.AddPlot(PlotData("hr", self.times, self.heartRates))
        #mp.AddPlot(PlotData("pwr/hr", self.times, self.hrpw))
        #mp.AddPlot(PlotData("pwr skewed", self.powerTime, self.power))
        
        mp.Plot()
        #sys.exit(0)
    
    def CalcHRPowerFactor(self, timeSkew):
        self.hrpw = [0] * len(self.times)
        self.powerTime = self.times[:]
        for i in range(len(self.powerTime)):
            self.powerTime[i] += timeSkew

        startIdx = None
        for i in range(len(self.times)):
            t = self.times[i]
            if t >= timeSkew:
                if not startIdx:
                    #print '%.4f > %d' % (t, timeSkew)
                    startIdx = i
                power = GetValueAtTime(self.power, self.powerTime, t)
                hr = self.heartRates[i]
                self.hrpw[i] = power / hr
            else:
                self.hrpw[i] = 0
        #print self.times[:10]
        #print '%d -> %d' % (timeSkew, startIdx)
        return startIdx
    
    def FindTimeSkew(self):
        smallestSdev = None
        smallestSdevAt = None
        for s in range(0, 91):
            startIdx = self.CalcHRPowerFactor(s)
            sdev = Sdev(self.hrpw[startIdx:])
            print '%d => %.5f' % (s, sdev)
            if not smallestSdev:
                smallestSdev = sdev
                smallestSdevAt = s
            elif sdev < smallestSdev:
                smallestSdev = sdev
                smallestSdevAt = s
        print 'Skew seems to be %d' % smallestSdevAt
        self.CalcHRPowerFactor(smallestSdevAt)
                
            
class LapInfo:
    def __init__(self, lapXml):
        
        self.StoreReportedData(lapXml)
        
        self.lastTime = 0
        self.startTime = 0
        self.lastDistance = 0
        
        self.times = []
        self.altitudes = []
        self.heartRates = []
        self.distances = []
    
    def StoreReportedData(self, lapXml):
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

        if len(self.times) > 2:
            wa = WorkOutAnalyzer(staticData, self.times, self.distances, self.altitudes, self.heartRates)
            wa.Analyze()
            print '      Analyzer joules / Calories = %f' % (wa.joules / self.reportedCalories if self.reportedCalories > 0 else 0)
            print '                        Calories = %f' % self.reportedCalories
            print '                     My Calories = %f' % (wa.joules / 640)
    
    def ProcessTrackPoint(self, trackPoint, activityStartTime, stopTime):
        activityTimeElapsed = trackPoint.time - activityStartTime
        if stopTime != 0 and activityTimeElapsed > stopTime:
            self.UpdateLapStats()
            return False
        if 0 == self.startTime:
            self.startTime = trackPoint.time
        else:
            elapsed = trackPoint.time - self.startTime
            intervalDistance = trackPoint.distance - self.lastDistance

            self.times.append(elapsed)
            self.heartRates.append(trackPoint.heartRate)
            self.altitudes.append(trackPoint.altitude)
            self.distances.append(intervalDistance)
        
        self.lastTime = trackPoint.time
        self.lastDistance = trackPoint.distance
        
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
            times = []
            distances = []
            altitudes = []
            heartRates = []
            print 'Activity'
            totalActivityTime = 0.0
            for lap in activity.laps:
                print '  Lap'
                staticData = StaticData(88.0, COEFF_FRIC_MTB, 0.5)
        
                if len(lap.times) > 2:
                    wa = WorkOutAnalyzer(staticData, lap.times, lap.distances, lap.altitudes, lap.heartRates)
                    wa.Analyze()
                    print '      Analyzer joules / Calories = %f' % (wa.joules / lap.reportedCalories if lap.reportedCalories > 0 else 0)
                    print '                        Calories = %f' % lap.reportedCalories
                    print '                     My Calories = %f' % (wa.joules / 640)
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