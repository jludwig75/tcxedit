import math

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


