import math

import sys
sys.path.append("D:/Zoo/Gibbon/vrpSolver")
import vrpSolver

from .common import *
from .error import *
from .ds import *

# History =====================================================================
# 20230613 - Independent from PDSTSPWI, gull, vrpSolver
#          - Now support multiple legs in the flight
# =============================================================================

# Description =================================================================
# This script calculates the optimal speed for a drone to make delivery under changing wind condition

# Some technical parameters for delivery drones ===============================
# [+] Antwork technology, Drone RA3
#    - Ref: https://xyi-web.oss-cn-hangzhou.aliyuncs.com/ADNET%20WHITE%20PAPER.pdf
#    - Max cruise speed: 60 km/h or 16.67 m/s
#    - Max flying time: ~45 min
#    - Max range: 25 km
#    - Empty weight: 6.8 kg
#    - Max payload: 6 kg
#    - Rain/snow proof
# [+] DJI Phantom 3 Pro (no longer available)
#    - Battery cap: 4480 mAh
#    - Battery voltage: 15.2 V
#    - Equivalent energy cap: 245145 J
# [+] DJI P4 Multispectral (Commercial version)
#    - Max ascent speed: 6 m/s
#    - Max landing speed: 3 m/s
#    - Max speed: 31 mph (P-mode)/36 mph (A-mode)
#    - Max flying time: ~27 min
#    - Ref: https://www.dji.com/p4-multispectral/specs
#    - Battery cap: 5870 mAh
#    - Battery voltage: 15.2 V
#    - Energy cap: 89.2 Wh (321120 J)

# Parameters for drone ========================================================
DRONE_MAX_GND_SPEED = 25 # [m/s] # From literature *
DRONE_LAUNCHING_SPEED = 10 # [m/s]
DRONE_LANDING_SPEED = 5 # [m/s]
DRONE_EMPTY_WEIGHT = 1.5 # [kg]
DRONE_PARCEL_WEIGHT = 2 # [kg]
DRONE_CRUISE_ALTITUDE = 50 # [m]
DRONE_BATTARY_CAPACITY = 400000 # [J]  # This number is reasonable as we can find 11000 mAh batteries

# Reference ===================================================================
# * Drone Deliveries Logistics, Efficiency, Safety and Last Mile Trade-offs
# Link: http://web.cecs.pdx.edu/~maf/Conference_Proceedings/2018_Drone_Deliveries_Logistics_Efficiency_Safety_Last_Mile_Trade-offs.pdf

# NOTE ========================================================================
# In this script, windDeg is the direction of the wind speed vector (start from the drone)
# In meteorology term, wind direction is the direction where the wind comes from

def droneLaunchEnergy(windSpd:float, windDeg: float, payload: float, launchSpd: float=DRONE_LAUNCHING_SPEED, cruiseAlt: float=DRONE_CRUISE_ALTITUDE)-> float:
    """Energy consumption of launching the drone"""
    return (cruiseAlt / launchSpd) * droneConsumptionRate(
        payload = payload, vertSpd = launchSpd, windSpd = windSpd, windDeg = windDeg, gndSpd = 0, gndDeg = 0)

def droneDropParcelEnergy(windSpd:float, windDeg: float, payload: float, hovering: float, launchSpd: float=DRONE_LAUNCHING_SPEED, landingSpd: float=DRONE_LANDING_SPEED, cruiseAlt: float=DRONE_CRUISE_ALTITUDE) -> float:
    """Energy consumption for hovering, landing and launch again"""
    erg = 0
    # Hovering energy
    erg += hovering * droneConsumptionRate(
        payload = payload, vertSpd = 0, windSpd = windSpd, windDeg = windDeg, gndSpd = 0, gndDeg = 0)
    # Landing energy
    erg += (cruiseAlt / landingSpd) * droneConsumptionRate(
        payload = payload, vertSpd = landingSpd, windSpd = windSpd, windDeg = windDeg, gndSpd = 0, gndDeg = 0)
    # Relaunching energy
    erg += (cruiseAlt / launchSpd) * droneConsumptionRate(
        payload = 0, vertSpd = launchSpd, windSpd = windSpd, windDeg = windDeg, gndSpd = 0, gndDeg = 0)
    return erg

def droneLandingEnergy(windSpd:float, windDeg: float, payload: float, landingSpd: float=DRONE_LANDING_SPEED, cruiseAlt: float=DRONE_CRUISE_ALTITUDE) -> float:
    """Energy consumption of landing the drone"""
    return (cruiseAlt / landingSpd) * droneConsumptionRate(
        payload = payload, vertSpd = landingSpd, windSpd = windSpd, windDeg = windDeg, gndSpd = 0, gndDeg = 0)

def droneTurnaroundDropRange(windSpd: float, windDeg: float, cruiseBat: float, payload: float, lod:int=30) -> dict:

    """Given consistent wind speed and wind direction, with given parcel weight, returns the drone range that\
        1) Drone can fly at maximum speed\
        2) Drone can make delivery
    
    windSpd:    "Consistent wind speed"
    windDeg:    "Consistent wind direction"
    cruiseBat:  "Drone energy for cruising"
    payload:    "Payload that needs to be delivered"
    lod: int, optional, default 30

    """

    # Initialize ==============================================================
    # If drone is flying at its maximum ground speed
    maxSpeedRange = []
    # If drone is flying at its maximum endurance
    maxDistRange = []

    # Get maxSpeedRange =======================================================
    def calMaxSpeedRange(deg):
        # Max speed consumption rate
        loadedMaxSpeedConsumptionRate = droneConsumptionRate(
            payload = payload,
            vertSpd = 0,
            windSpd = windSpd,
            windDeg = windDeg,
            gndSpd = DRONE_MAX_GND_SPEED,
            gndDeg = deg)
        noLoadMaxSpeedConsumptionRate = droneConsumptionRate(
            payload = 0,
            vertSpd = 0,
            windSpd = windSpd,
            windDeg = windDeg,
            gndSpd = DRONE_MAX_GND_SPEED,
            gndDeg = deg + 180)

        # Assuming maximum ground speed, calculate the maximum endurance
        maxOneWayTime = cruiseBat / (loadedMaxSpeedConsumptionRate + noLoadMaxSpeedConsumptionRate)
        r = maxOneWayTime * DRONE_MAX_GND_SPEED
        return r

    # Get maxDistRange ========================================================
    def calMaxDistRange(deg):
        # Search on the percentage of allocating energy on Go/back
        pLB = 0
        pUB = 1
        p = 0.5
        r = 0
        while (pUB - pLB > 0.001):
            goMaxEndur = droneMaxEndurance(
                payload = payload,
                cruiseBat = (1 - p) * cruiseBat,
                windSpd = windSpd,
                windDeg = windDeg,
                gndDeg = deg)['maxDist']
            backMaxEndur = droneMaxEndurance(
                payload = 0,
                cruiseBat = p * cruiseBat,
                windSpd = windSpd,
                windDeg = windDeg,
                gndDeg = deg + 180)['maxDist']
            if (goMaxEndur > backMaxEndur):
                pLB = p               
            else:
                pUB = p
            p = pLB + (pUB - pLB) / 2
            r = min(goMaxEndur, backMaxEndur)
        return r

    # Poly ====================================================================
    for d in range(lod):
        maxSpeedRange.append(calMaxSpeedRange(d * 360 / lod))
        maxDistRange.append(calMaxDistRange(d * 360 / lod))

    return {
        'maxSpeedRange': maxSpeedRange,
        'maxDistRange': maxDistRange
    }

def optDroneDropSpeed(
    launchPt: pt,
    waypoints: list[pt], 
    payloadDrops: list[float],        
    windSpd: float, 
    windDeg: float, 
    cruiseBat: float, 
    returnPt: pt|None = None, 
    hoveringBeforeDropping: float|int|list|None = None,
    hoveringAfterDropping: float|int|list|None = None,
    maxGndSpd: float = DRONE_MAX_GND_SPEED, 
    droneEmptyWeight: float = DRONE_EMPTY_WEIGHT,
    spdPrecise: float=0.1) -> dict:

    """Given wind and drone battary conditions, optimize the ground speed of legs between waypoints.

    Notes
    -----
    In this function, the energy consumption for launching and landing are not included/considered.


    Parameters
    ----------
    waypoints: list[pt], required
        A list of waypoints to be visited by drone, could be 

    """

    # Sanity check ============================================================
    if (len(waypoints) != len(payloadDrops)):
        raise UnsupportedInputError("ERROR: the length of `waypoints` should be the same as the length of `payloadDrops`")

    if (type(hoveringBeforeDropping) == float or type(hoveringBeforeDropping) == int):
        hoveringBeforeDropping = [hoveringBeforeDropping] * len(waypoints)
    if (type(hoveringAfterDropping) == float or type(hoveringAfterDropping) == int):
        hoveringAfterDropping = [hoveringAfterDropping] * len(waypoints)

    if (returnPt == None):
        returnPt = launchPt
     
    # Legs between waypoints
    gndSpdList = []
    gndDegList = []
    gndDistList = []

    # Function of calculating the energy consumption given ground speeds
    def calEnergy(gndSpdList, gndDegList, gndDistList) -> float:
        ergy = 0
        # Energy for cruising
        for i in range(len(waypoints)):
            ergy += (gndDistList[i] / gndSpdList[i]) * droneConsumptionRate(
                payload = sum([payloadDrops[k] for k in range(i, len(waypoints))]), 
                windSpd = windSpd, windDeg = windDeg, vertSpd = 0, 
                gndSpd = gndSpdList[i], gndDeg = gndDegList[i], droneEmptyWeight = droneEmptyWeight)
        ergy += (gndDistList[-1] / gndSpdList[-1]) * droneConsumptionRate(
            payload = 0, windSpd = windSpd, windDeg = windDeg, vertSpd = 0, 
            gndSpd = gndSpdList[-1], gndDeg = gndDegList[-1], droneEmptyWeight = droneEmptyWeight)
        # Energy for hovering - loaded with package
        if (isinstance(hoveringBeforeDropping, list)):
            for i in range(len(waypoints)):
                ergy += hoveringBeforeDropping[i] * droneConsumptionRate(
                    payload = sum([payloadDrops[k] for k in range(i, len(waypoints))]), 
                    windSpd = windSpd, windDeg = windDeg, vertSpd = 0, 
                    gndSpd = 0, gndDeg = 0, droneEmptyWeight = droneEmptyWeight)
        # Energy for hovering - unloaded the package
        if (isinstance(hoveringAfterDropping, list)):
            for i in range(len(waypoints)):
                ergy += hoveringAfterDropping[i] * droneConsumptionRate(
                    payload = sum([payloadDrops[k] for k in range(i + 1, len(waypoints))]), 
                    windSpd = windSpd, windDeg = windDeg, vertSpd = 0, 
                    gndSpd = 0, gndDeg = 0, droneEmptyWeight = droneEmptyWeight)
        return ergy

    def calProfile(gndSpdList, gndDistList) -> dict:
        totalPayload = sum(payloadDrops)
        timeStamps = [0.0]
        visitSeq = [launchPt]
        payloadSeq = [totalPayload]        
        if (hoveringBeforeDropping == None and hoveringAfterDropping == None):
            for i in range(len(waypoints)):
                timeStamps.append(timeStamps[-1] + gndDistList[i] / gndSpdList[i])
                visitSeq.append(waypoints[i])
                payloadSeq.append(sum([payloadDrops[k] for k in range(i + 1, len(waypoints))]))
        elif (isinstance(hoveringBeforeDropping, list) and hoveringAfterDropping == None):
            for i in range(len(waypoints)):
                timeStamps.append(timeStamps[-1] + gndDistList[i] / gndSpdList[i])
                visitSeq.append(waypoints[i])
                payloadSeq.append(sum([payloadDrops[k] for k in range(i, len(waypoints))]))
                timeStamps.append(timeStamps[-1] + hoveringBeforeDropping[i])
                visitSeq.append(waypoints[i])
                payloadSeq.append(sum([payloadDrops[k] for k in range(i + 1, len(waypoints))]))
        elif (hoveringBeforeDropping == None and isinstance(hoveringAfterDropping, list)):
            for i in range(len(waypoints)):
                timeStamps.append(timeStamps[-1] + gndDistList[i] / gndSpdList[i])
                visitSeq.append(waypoints[i])
                payloadSeq.append(sum([payloadDrops[k] for k in range(i + 1, len(waypoints))]))
                timeStamps.append(timeStamps[-1] + hoveringAfterDropping[i])
                visitSeq.append(waypoints[i])
                payloadSeq.append(sum([payloadDrops[k] for k in range(i + 1, len(waypoints))]))
        elif (isinstance(hoveringBeforeDropping, list) and isinstance(hoveringAfterDropping, list)):
            for i in range(len(waypoints)):
                timeStamps.append(timeStamps[-1] + gndDistList[i] / gndSpdList[i])
                visitSeq.append(waypoints[i])
                payloadSeq.append(sum([payloadDrops[k] for k in range(i, len(waypoints))]))
                timeStamps.append(timeStamps[-1] + hoveringBeforeDropping[i])
                visitSeq.append(waypoints[i])
                payloadSeq.append(sum([payloadDrops[k] for k in range(i, len(waypoints))]))
                timeStamps.append(timeStamps[-1] + hoveringAfterDropping[i])
                visitSeq.append(waypoints[i])
                payloadSeq.append(sum([payloadDrops[k] for k in range(i + 1, len(waypoints))]))
        timeStamps.append(timeStamps[-1] + gndDistList[-1] / gndSpdList[-1])
        visitSeq.append(returnPt)
        payloadSeq.append(0)
        return {
            'timeStamps': timeStamps,
            'visitSeq': visitSeq,
            'payloadSeq': payloadSeq,
            'profileTime': timeStamps[-1]
        }

    # Step 1: Initialize, assume that the drone is flying at its maximum speed, regardless of energy limit
    gndSpdList.append(maxGndSpd)
    gndDegList.append(vrpSolver.headingXY(launchPt, waypoints[0]))
    gndDistList.append(vrpSolver.distEuclidean2D(launchPt, waypoints[0]))
    for i in range(len(waypoints) - 1):
        gndSpdList.append(maxGndSpd)
        gndDegList.append(vrpSolver.headingXY(waypoints[i], waypoints[i + 1]))
        gndDistList.append(vrpSolver.distEuclidean2D(waypoints[i], waypoints[i + 1]))
    gndSpdList.append(maxGndSpd)
    gndDegList.append(vrpSolver.headingXY(waypoints[-1], returnPt))
    gndDistList.append(vrpSolver.distEuclidean2D(waypoints[-1], returnPt))

    # Now, ergy is the energy needed to complete the flight in maximum ground speed
    ergy = calEnergy(gndSpdList, gndDegList, gndDistList)
    profile = calProfile(gndSpdList, gndDistList)
    # If the energy to fly at maximum speed is already sufficient, returns max speed
    if (ergy <= cruiseBat):
        return {
            'feasible': True,
            'gndSpdList': gndSpdList,
            'remainBat': cruiseBat - ergy,
            'timeStamps': profile['timeStamps'],
            'visitSeq': profile['visitSeq'],
            'payloadSeq': profile['payloadSeq'],
            'profileTime': profile['profileTime']
        }

    # Step 2: Reduce energy consumption, this step is done by reducing ground speeds
    # NOTE: Purpose of this step is to find a feasible solution
    while (True):
        print(ergy, gndSpdList)
        bestReducedLegIndex = len(gndSpdList)
        for i in range(len(gndSpdList)):
            if (gndSpdList[i] > spdPrecise):
                updateGndSpd = [spd for spd in gndSpdList]
                updateGndSpd[i] -= spdPrecise
                updatedEnergy = calEnergy(updateGndSpd, gndDegList, gndDistList)
                if (updatedEnergy < ergy):
                    ergy = updatedEnergy
                    bestReducedLegIndex = i
        if (bestReducedLegIndex == len(gndSpdList)):
            break
        else:
            gndSpdList[bestReducedLegIndex] -= spdPrecise
            gndSpdList[bestReducedLegIndex] = round(gndSpdList[bestReducedLegIndex], 2)

    # If after reducing energy consumption, it is still greater than given limit, then its infeasible
    if (ergy > cruiseBat):
        return {
            'feasible': False,
            'gndSpdList': [],
            'remainBat': 0,
            'timeStamps': [],
            'visitSeq': [],
            'payloadSeq': [],
            'profileTime': 0
        }
    # else:
    #     profile = calProfile(gndSpdList, gndDistList)
    #     return {
    #         'feasible': True,
    #         'gndSpdList': gndSpdList,
    #         'remainBat': cruiseBat - ergy,
    #         'timeStamps': profile['timeStamps'],
    #         'visitSeq': profile['visitSeq'],
    #         'payloadSeq': profile['payloadSeq'],
    #         'profileTime': profile['profileTime']
    #     }

    # Step 3: When we find a solution which the energy consumption is within cruiseBat, improve gndSpds
    # NOTE: Improving direction is the best \Delta Energy / \Delta Time

    profileTime = calProfile(gndSpdList, gndDistList)['profileTime']
    while (True):
        print(ergy, gndSpdList)
        marginImprove = []
        for i in range(len(gndSpdList)):
            if(gndSpdList[i] + spdPrecise <= maxGndSpd):
                updateGndSpd = [spd for spd in gndSpdList]
                updateGndSpd[i] += spdPrecise
                updatedEnergy = calEnergy(updateGndSpd, gndDegList, gndDistList)
                if (updatedEnergy < cruiseBat):
                    updatedProfileTime = calProfile(updateGndSpd, gndDistList)['profileTime']
                    marginImprove.append(((profileTime - updatedProfileTime) / (updatedEnergy - ergy), i, updatedProfileTime, updatedEnergy))
        if (len(marginImprove) == 0):
            break
        else:
            m = max(marginImprove)
            profileTime = m[2]
            gndSpdList[m[1]] += spdPrecise
            gndSpdList[m[1]] = round(gndSpdList[m[1]], 2)
            ergy = m[3]

    profile = calProfile(gndSpdList, gndDistList)
    return {
        'feasible': True,
        'gndSpdList': gndSpdList,
        'remainBat': cruiseBat - ergy,
        'timeStamps': profile['timeStamps'],
        'visitSeq': profile['visitSeq'],
        'payloadSeq': profile['payloadSeq'],
        'profileTime': profile['profileTime']
    }

def droneMaxEndurance(payload:float, cruiseBat: float, windSpd: float, windDeg: float, gndDeg: float) -> dict:
    
    """Given parameters, return the most energy saving speed and max endurance
    
    payload:    "Weight in [kg]" = None,
    cruiseBat:  "Remaining battery" = 500000,
    windSpd:    "Wind speed" = 0,
    windDeg:    "Direction of wind speed, in [degree]" = 0,
    gndDeg:     "Direction of ground speed, in [degree]" = 0    
    """

    # bounding of speed =======================================================
    left = 0
    right = DRONE_MAX_GND_SPEED
    while (right - left > 0.01):
        m1 = left + (right - left) / 3
        m2 = right - (right - left) / 3
        m1Consumption = droneConsumptionRate(
            payload = payload,
            vertSpd = 0,
            windSpd = windSpd,
            windDeg = windDeg,
            gndSpd = m1,
            gndDeg = gndDeg)
        m2Consumption = droneConsumptionRate(
            payload = payload,
            vertSpd = 0,
            windSpd = windSpd,
            windDeg = windDeg,
            gndSpd = m2,
            gndDeg = gndDeg)
        m1Endur = (cruiseBat / m1Consumption) * m1
        m2Endur = (cruiseBat / m2Consumption) * m2
        if (m1Endur >= m2Endur):
            right = m2
        else:
            left = m1        
    savGndSpd = left + (right - left) / 2
    savConsumption = droneConsumptionRate(
        payload = payload,
        vertSpd = 0,
        windSpd = windSpd,
        windDeg = windDeg,
        gndSpd = savGndSpd,
        gndDeg = gndDeg)
    savEndur = (cruiseBat / savConsumption) * savGndSpd

    return {
        'savGndSpd': savGndSpd,
        'maxDist': savEndur
    }

def queryWindSpd(wind:dict|IntervalTree, now:float, cycleFlag:bool=True) -> dict|None:
    
    """Given a time stamp, find current wind speed, wind direction, time left in current wind window"

    wind:       "List of dictionary, wind speed/wind direct in time sequence, time starts from 0\
                [{\
                    'startTime': startTime,\
                    'endTime': endTime,\
                    'windSpd': windSpd, # [m/s]\
                    'windDeg': winDeg\
                }, ...]" = None,
    now:        "Current time" = None,
    cycleFlag:  "True if repeat the wind data in cycle" = True

    """

    # A polynomial variation in wind speed with height, widely used in wind turbine engineering
    # v_w(h) = v_{10} (\frac{h}{h_{10}})^\alpha
    # - v_w(h): wind speed at AGL h
    # - v_{10}: wind at AGL 10 m
    # - h: height in AGL [m]
    # - h_{10}: 10 m (Ground level)
    # - \alpha: Hellmann exponent

    # Initialize ==============================================================
    curWindSpd = None
    curWindDeg = None
    timeLeft = None

    if (isinstance(wind, dict)):
        lengWindData = 0
        for w in wind:
            if (w['endTime'] > lengWindData):
                lengWindData = w['endTime']
        if (cycleFlag):
            while (now >= lengWindData):
                now -= lengWindData
        else:
            if (now > lengWindData):
                return None

        for s in range(len(wind)):
            if (now >= wind[s]['startTime'] and now < wind[s]['endTime']):
                curWindSpd = wind[s]['windSpd']
                # NOTICE: The direction of the wind is the same as the wind vector, not meteorology term
                curWindDeg = wind[s]['windDeg']
                timeLeft = wind[s]['endTime'] - now
                if (curWindSpd != None and curWindDeg != None and timeLeft != None):
                    return {
                        'curWindSpd': curWindSpd,
                        'curWindDeg': curWindDeg,
                        'timeLeft': timeLeft
                    }
    return None

def droneConsumptionRate(payload:float, windSpd:float, windDeg:float, vertSpd:float, gndSpd:float, gndDeg:float, droneEmptyWeight:float=DRONE_EMPTY_WEIGHT)->float:

    """Drone energy consumption rate in [J/s]

    payload:    "Weight in [kg]" = None,
    windSpd:    "Wind speed" = 0,
    windDeg:    "Direction of wind speed, in [degree]" = 0,
    vertSpd:    "Vertical speed" = 0,
    gndSpd:     "Ground speed of drone" = 0,
    gndDeg:     "Direction of gnd speed, in [degree]" = 0    
    """

    # Reference ===============================================================
    # Z. Liu et al. A Power Consumption Model for Multi-rotor Small Unmanned Aircraft Systems
    # Following three parts of power are considered in the model
    # P_i: Induced Power -> Function of (Thrust, V_vert)
    # P_p: Profile Power -> Function of (Thrust, V_air)
    # P_par: Parasite Power -> Function of (V_air)
    # For thrust -> Function of (payload, V_air)

    # Parameters used in Liu's model ==========================================
    k1    = 0.8554
    k2    = 0.3051 # (kg/m)^(1/2)
    c1    = 2.8037 # (m/kg)^(1/2)
    c2    = 0.3177 # (m/kg)^(1/2)
    c3    = 0
    c4    = 0.0296 # kg/m
    c5    = 0.0279 # Ns/m
    c6    = 0
    g     = 9.8    # m/(s^2)
    alpha = 10     # degree

    # Calculate air speed =====================================================
    # V_air + V_wind = V_gnd (as vectors)
    gndX, gndY = vrpSolver.vecPolar2XY([gndSpd, gndDeg])
    windX, windY = vrpSolver.vecPolar2XY([windSpd, windDeg])
    airX = gndX - windX
    airY = gndY - windY
    airSpd, _ = vrpSolver.vecXY2Polar([airX, airY])

    # Induced power, profile power, parasite power ============================
    # Power = induced power + profile power + parasite power
    # Thrust
    T = math.sqrt(
        ((droneEmptyWeight + payload) * g - c5 * (airSpd * math.cos(math.radians(alpha)))**2)**2 
        + (c4 * airSpd * airSpd)**2)
    # Induced power, c6 term has been neglected since c6 = 0
    powerInduced = k1 * T * (vertSpd / 2 + math.sqrt((vertSpd / 2)**2 + T / (k2**2)))
    # Profile power, c3 term has been neglected since c3 = 0
    powerProfile = c2 * (T**1.5)
    # Parasite power
    powerParasite = c4 * (airSpd**3)
    # Power
    power = powerInduced + powerProfile + powerParasite

    return power
