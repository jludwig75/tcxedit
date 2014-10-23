import xml.etree.ElementTree as ET
import math
import time


TIME_FORMAT_STRING = '%Y-%m-%dT%H:%M:%SZ'
NS = '{http://www.garmin.com/xmlschemas/TrainingCenterDatabase/v2}'

def TAG_NAME(name):
    return NS + name

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
        
class TrackPoint:
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

class Lap:
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
    def __init__(self, activityXml):
        self.id = activityXml.find(TAG_NAME('Id')).text
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
        self.currentActivity = Activity(activityXml)
    
    def VisitLap(self, lapXml):
        self.currentLap = Lap(lapXml)
    
    def VisitTackPoint(self, trackPointXml):
        try:
            trackPoint = TrackPoint(trackPointXml)
            if self.lastActivityStartTime == 0:
                self.lastActivityStartTime = trackPoint.time
            return self.currentLap.ProcessTrackPoint(trackPoint, self.lastActivityStartTime, self.stopTime)
        except TackPointDataMissing as e:
            return True
            


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


class TcxFile:
    def __init__(self, elementTree, activities):
        self.elementTree = elementTree
        self.activities = activities
    
    @staticmethod
    def ParseFile(fileName, stopTime):
        tree = ET.parse(fileName)
    
        ET.register_namespace('', 'http://www.garmin.com/xmlschemas/ActivityExtension/v2')
        ET.register_namespace('', 'http://www.garmin.com/xmlschemas/TrainingCenterDatabase/v2')
        
        root = tree.getroot()
        
        visitor = TcxVisitor(stopTime)

        VisitTack(root, visitor)
    
        return TcxFile(tree, visitor.activities)

    def Write(self, fileName):
        return elementTree.write(fileName, xml_declaration=True,encoding='utf-8', method="xml")