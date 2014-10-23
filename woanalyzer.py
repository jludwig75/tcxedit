import math

from calc import *
from filter import *
from stats import *
from plot import *


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
        self.altitudes = SmoothData(self.altitudes, GAUSSIAN_SMOOTH_SDEV)
        self.heartRates = SmoothData(self.heartRates, GAUSSIAN_SMOOTH_SDEV)
        self.distances = SmoothData(self.distances, GAUSSIAN_SMOOTH_SDEV)
        
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
        self.speeds = SmoothData(self.speeds, GAUSSIAN_SMOOT_SECONDARY_DATA)
        # Smooth the slopes
        self.slopes = SmoothData(self.slopes, GAUSSIAN_SMOOT_SECONDARY_DATA)
    
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
        #self.fd = SmoothData(self.fd, GAUSSIAN_SMOOT_SECONDARY_DATA)
        #self.ft = SmoothData(self.ft, GAUSSIAN_SMOOT_SECONDARY_DATA)
        #self.fs = SmoothData(self.fs, GAUSSIAN_SMOOT_SECONDARY_DATA)
        #self.ff = SmoothData(self.ff, GAUSSIAN_SMOOT_SECONDARY_DATA)
        
        #self.fr = SmoothData(self.fr, GAUSSIAN_SMOOT_SECONDARY_DATA)
        
        self.CalculateRiderForce()
        
        self.CalculateWorkAndPower()
    
    def PostProcess(self):
        # Smooth the power
        self.power = SmoothData(self.power, GAUSSIAN_SMOOT_TERTIARY_DATA)
        for i in range(len(self.power) - 8, len(self.power)):
            self.power[i] = 0
        
        #self.CalcHRPowerFactor(30)
        
        #self.FindTimeSkew()
        
        #self.power = SmoothData(self.power, GAUSSIAN_SMOOTH_SDEV)
        #self.heartRates = SmoothData(self.heartRates, GAUSSIAN_SMOOTH_SDEV)
        #self.speeds = SmoothData(self.speeds, GAUSSIAN_SMOOTH_SDEV)
        
        
        self.pedalingPower = [x if x > 0 else 0 for x in self.power]
        
        pedalingJoules = 0
        timePedaling = 0
        lastTime = 0
        hrTime = 0
        for i in range(len(self.pedalingPower)):
            if self.pedalingPower[i] > 0:
                elapsed = self.times[i] - lastTime
                timePedaling += elapsed
                pedalingJoules += elapsed * self.power[i]
                hrTime += elapsed * self.heartRates[i]
            lastTime = self.times[i]
            
        self.avgPedalingPower = pedalingJoules / timePedaling
        self.avgHr = hrTime / timePedaling
        
        self.pedalingPowerTrend = SmoothData(self.pedalingPower, 20.0)
        self.hrTrend = SmoothData(self.heartRates, 20.0)
        
        self.pwrToHr = [self.pedalingPowerTrend[i] / self.hrTrend[i] for i in range(len(self.times))]
    
    def Plot(self, options):
        mp = MuliPlotter(10000)
        
        if options.DoHeartRatePlot():
            mp.AddPlot(PlotData("hr", self.times, self.heartRates))
        if options.DoAltitudePlot():
            mp.AddPlot(PlotData("altitude", self.times, self.altitudes))
        if options.DoSpeedPlot():
            mp.AddPlot(PlotData("speed", self.times, self.speeds))
        if options.DoPedalingPowerPlot():
            mp.AddPlot(PlotData("pedaling power", self.times, self.pedalingPower))
        if options.DoHeartRatePlot() and options.DoTrendPlots():
            mp.AddPlot(PlotData("hr trend", self.times, self.hrTrend))
        if options.DoPowerPlot() and options.DoTrendPlots():
            mp.AddPlot(PlotData("pedaling power trend", self.times, self.pedalingPowerTrend))
        if options.DoPwrToHrPlot():
            mp.AddPlot(PlotData("power to HR", self.times[MAX_FILTER_WIDTH / 2:-MAX_FILTER_WIDTH / 2], self.pwrToHr[MAX_FILTER_WIDTH / 2:-MAX_FILTER_WIDTH / 2]))
        if options.DoPowerPlot():
            mp.AddPlot(PlotData("power", self.times, self.power))
        if options.DoGradePlot():
            mp.AddPlot(PlotData("slope", self.times, self.slopes))
        if options.DoDistancePlots():
            mp.AddPlot(PlotData("distance", self.times, self.distances))
        if options.DoForcePlots():
            mp.AddPlot(PlotData("total force", self.times, self.ft))
            mp.AddPlot(PlotData("slope force", self.times, self.fs))
            mp.AddPlot(PlotData("drag", self.times, self.fd))
            mp.AddPlot(PlotData("Fr", self.times, self.ft))
            mp.AddPlot(PlotData("Ft", self.times, self.fr))
        if options.DoJoulesPlot():
            mp.AddPlot(PlotData("work", self.times, self.joulesCum))
        #mp.AddPlot(PlotData("pwr skewed", self.powerTime, self.power))
        
        mp.Plot()
        
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
                    startIdx = i
                power = GetValueAtTime(self.power, self.powerTime, t)
                hr = self.heartRates[i]
                self.hrpw[i] = power / hr
            else:
                self.hrpw[i] = 0
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

