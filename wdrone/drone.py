import math
from itertools import product

import sys
sys.path.append("D:/Zoo/Gibbon/vrpSolver")
import vrpSolver
from vrpSolver.common import *

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

# The time of each step of drone delivery =====================================
#    - With payload - Launching from depot
#    - With payload - Cruise to customer
#    - With payload - Hover at customer (could be zero) 
#    - With payload - Landing at customer (could be zero)
#    - Without payload - Launching from customer (could be zero)
#    - Without payload - Cruise to depot
#    - Without payload - Landing at depot

# Parameters for drone ========================================================
DRONE_SPEED_LIMIT = 10 # [m/s]
DRONE_MIN_GND_SPEED = 10 # [m/s]
DRONE_MAX_GND_SPEED = 25 # [m/s] # From literature *
DRONE_TAKEOFF_SPEED = 10 # [m/s]
DRONE_LANDING_SPEED = 5 # [m/s]
DRONE_EMPTY_WEIGHT = 1.5 # [kg]
DRONE_PARCEL_WEIGHT = 2 # [kg]
DRONE_CRUISE_ALTITUDE = 50 # [m]
DRONE_BATTARY_CAPACITY = 400000 # [J]  # This number is reasonable as we can find 11000 mAh batteries
DRONE_LANDING_FLAG = True # True if we need to land at the customer location

# Reference
# * Drone Deliveries Logistics, Efficiency, Safety and Last Mile Trade-offs
# Link: http://web.cecs.pdx.edu/~maf/Conference_Proceedings/2018_Drone_Deliveries_Logistics_Efficiency_Safety_Last_Mile_Trade-offs.pdf

# NOTE ========================================================================
# In this script, windDeg is the direction of the wind speed vector (start from the drone)
# In meteorology term, wind direction is the direction where the wind comes from

def profileOptTurnaround(
    startLoc: pt,
    cusLoc: pt,
    endLoc: pt,
    hoverTime: float = 0,
    serviceTime: float = 0,
    landFlag: bool = DRONE_LANDING_FLAG,
    payload: float = DRONE_PARCEL_WEIGHT,
    batCap: float = DRONE_BATTARY_CAPACITY, 
    windSpd: float = 0,
    windDeg: float = 0) -> dict:

    # Ground speed degree =====================================================
    gndDegToCustomer = vrpSolver.headingLatLon(startLoc, cusLoc)
    gndDegToEndLoc = vrpSolver.headingLatLon(cusLoc, endLoc)
    gndDistToCustomer = vrpSolver.distLatLon(startLoc, cusLoc, distUnit='meter')
    gndDistToEndLoc = vrpSolver.distLatLon(cusLoc, endLoc, distUnit='meter')

    # Initialize ==============================================================
    curGndSpdToCus = None
    curGndSpdFromCus = None
    curTime = None
    # optPath = []

    # Find an initial feasible solution =======================================
    # Grid searching
    gridSize = 128 # Initially define grid size as 128 x 128, so that each time when dividing 2, coordinate will be integer
    # (0, 0) = [max, max] -> (128, 128) = [min, min]
    # Save the coordinates that has searched
    searchedGrid = []
    foundFeasibleFlag = False
    while (not foundFeasibleFlag and gridSize >= 1):
        # Search in grids
        for i, j in product(range(int(128 / gridSize) + 1), range(int(128 / gridSize) + 1)):
            gridCoord = (i * gridSize, j * gridSize)
            if (gridCoord not in searchedGrid):
                curGndSpdToCus = DRONE_MAX_GND_SPEED - i * gridSize / 128.0 * (DRONE_MAX_GND_SPEED - DRONE_MIN_GND_SPEED)
                curGndSpdFromCus = DRONE_MAX_GND_SPEED - j * gridSize / 128.0 * (DRONE_MAX_GND_SPEED - DRONE_MIN_GND_SPEED)
                curTime = profileTurnAround(
                    startLoc = startLoc,
                    cusLoc = cusLoc,
                    endLoc = endLoc,
                    hoverTime = hoverTime,
                    landFlag = landFlag,  
                    payload = payload,
                    batCap = batCap,
                    gndSpdToCus = curGndSpdToCus,
                    gndSpdFromCus = curGndSpdFromCus,
                    wind = wind,
                    launchTime = launchTime)
                if (curTime != None):
                    foundFeasibleFlag = True
                    break
                else:                    
                    searchedGrid.append(gridCoord)
        # If not found, use smaller grid
        gridSize = int(gridSize / 2)

    if (not foundFeasibleFlag):
        return {
            'gndSpdToCus': None,
            'gndSpdFromCus': None,
            'timeToCus': None,
            'timeFromCus': None,
            'totalTime': None
        }

    # Greedy search after finding feasible solution ===========================
    # NOTE: Of course this can be written in a compact way, this is just for simplicity
    stepLength = 0.01
    stopCriteria = False
    while (not stopCriteria):
        d = {}
        # Direction 2
        if (curGndSpdToCus + stepLength <= DRONE_MAX_GND_SPEED
            and curGndSpdFromCus - stepLength <= DRONE_MAX_GND_SPEED):
            cTime = profileTurnAround(
                startLoc = startLoc,
                cusLoc = cusLoc,
                endLoc = endLoc,
                hoverTime = hoverTime,
                landFlag = landFlag,
                payload = payload,
                batCap = batCap,
                gndSpdToCus = curGndSpdToCus + stepLength,
                gndSpdFromCus = curGndSpdFromCus - stepLength,
                wind = wind,
                launchTime = launchTime)
            if (cTime != None and cTime < curTime):
                d[2] = cTime
        # Direction 3
        if (curGndSpdToCus + stepLength <= DRONE_MAX_GND_SPEED):
            cTime = profileTurnAround(
                startLoc = startLoc,
                cusLoc = cusLoc,
                endLoc = endLoc,
                hoverTime = hoverTime,
                landFlag = landFlag,
                payload = payload,
                batCap = batCap,
                gndSpdToCus = curGndSpdToCus + stepLength,
                gndSpdFromCus = curGndSpdFromCus,
                wind = wind,
                launchTime = launchTime)
            if (cTime != None and cTime < curTime):
                d[3] = cTime
        # Direction 4
        if (curGndSpdToCus + stepLength <= DRONE_MAX_GND_SPEED
            and curGndSpdFromCus + stepLength <= DRONE_MAX_GND_SPEED):
            cTime = profileTurnAround(
                startLoc = startLoc,
                cusLoc = cusLoc,
                endLoc = endLoc,
                hoverTime = hoverTime,
                landFlag = landFlag,
                payload = payload,
                batCap = batCap,
                gndSpdToCus = curGndSpdToCus + stepLength,
                gndSpdFromCus = curGndSpdFromCus + stepLength,
                wind = wind,
                launchTime = launchTime)
            if (cTime != None and cTime < curTime):
                d[4] = cTime
        # Direction 5
        if (curGndSpdFromCus + stepLength <= DRONE_MAX_GND_SPEED):
            cTime = profileTurnAround(
                startLoc = startLoc,
                cusLoc = cusLoc,
                endLoc = endLoc,
                hoverTime = hoverTime,
                landFlag = landFlag,
                payload = payload,
                batCap = batCap,
                gndSpdToCus = curGndSpdToCus,
                gndSpdFromCus = curGndSpdFromCus + stepLength,
                wind = wind,
                launchTime = launchTime)
            if (cTime != None and cTime < curTime):
                d[5] = cTime
        # Direction 6
        if (curGndSpdToCus - stepLength <= DRONE_MAX_GND_SPEED
            and curGndSpdFromCus + stepLength <= DRONE_MAX_GND_SPEED):
            cTime = profileTurnAround(
                startLoc = startLoc,
                cusLoc = cusLoc,
                endLoc = endLoc,
                hoverTime = hoverTime,
                landFlag = landFlag,
                payload = payload,
                batCap = batCap,
                gndSpdToCus = curGndSpdToCus - stepLength,
                gndSpdFromCus = curGndSpdFromCus + stepLength,
                wind = wind,
                launchTime = launchTime)
            if (cTime != None and cTime < curTime):
                d[6] = cTime
        if (len(d) > 0):
            idx = min(d, key=d.get)
            if (idx == 2):
                curGndSpdToCus = curGndSpdToCus + stepLength
                curGndSpdFromCus = curGndSpdFromCus - stepLength
                curTime = d[2]
            elif (idx == 3):
                curGndSpdToCus = curGndSpdToCus + stepLength
                curGndSpdFromCus = curGndSpdFromCus
                curTime = d[3]
            elif (idx == 4):
                curGndSpdToCus = curGndSpdToCus + stepLength
                curGndSpdFromCus = curGndSpdFromCus + stepLength
                curTime = d[4]
            elif (idx == 5):
                curGndSpdToCus = curGndSpdToCus
                curGndSpdFromCus = curGndSpdFromCus + stepLength
                curTime = d[5]
            elif (idx == 6):
                curGndSpdToCus = curGndSpdToCus - stepLength
                curGndSpdFromCus = curGndSpdFromCus + stepLength
                curTime = d[6]
        else:
            stopCriteria = True
    return {
        'gndSpdToCus': curGndSpdToCus,
        'gndSpdFromCus': curGndSpdFromCus,
        'timeToCus': (
            gndDistToCustomer / curGndSpdToCus 
            + DRONE_CRUISE_ALTITUDE / DRONE_TAKEOFF_SPEED 
            + DRONE_CRUISE_ALTITUDE / DRONE_LANDING_SPEED),
        'timeAtCus': hoverTime,
        'timeFromCus': (
            gndDistToEndLoc / curGndSpdFromCus
            + DRONE_CRUISE_ALTITUDE / DRONE_TAKEOFF_SPEED 
            + DRONE_CRUISE_ALTITUDE / DRONE_LANDING_SPEED),
        'totalTime': (
            gndDistToCustomer / curGndSpdToCus 
            + gndDistToEndLoc / curGndSpdFromCus 
            + 2 * DRONE_CRUISE_ALTITUDE / DRONE_TAKEOFF_SPEED 
            + hoverTime 
            + 2 * DRONE_CRUISE_ALTITUDE / DRONE_LANDING_SPEED)
    }


def profileTurnaround(
    startLoc: pt = None,
    cusLoc: pt = None,
    endLoc: pt = None,
    hoverTime: float = 0,
    serviceTime: float = 0,
    landFlag: bool = DRONE_LANDING_FLAG,
    payload: float = 0,
    batCap: float = DRONE_BATTARY_CAPACITY, 
    gndSpdToCus: float = None,
    gndSpdFromCus: float = None,
    windSpd: float = None,
    windDeg: float = None
    ):

    # Ground speed degree =====================================================
    gndDegToCustomer = vrpSolver.headingLatLon(startLoc, cusLoc)
    gndDegToEndLoc = vrpSolver.headingLatLon(cusLoc, endLoc)
    gndDistToCustomer = vrpSolver.distLatLon(startLoc, cusLoc, distUnit='meter')
    gndDistToEndLoc = vrpSolver.distLatLon(cusLoc, endLoc, distUnit='meter')

    # Mission profile =========================================================
    profile = {
        'launchFromStartLoc' : {'startTime': None, 'endTime': None, 'batRemain': None},
        'cruiseToCustomer'   : {'startTime': None, 'endTime': None, 'batRemain': None},
        'hoverAtCustomer'    : {'startTime': None, 'endTime': None, 'batRemain': None},
        'landAtCustomer'     : {'startTime': None, 'endTime': None, 'batRemain': None},
        'waitingAtCustomer'  : {'startTime': None, 'endTime': None, 'batRemain': None},
        'launchFromCustomer' : {'startTime': None, 'endTime': None, 'batRemain': None},
        'cruiseToEndLoc'     : {'startTime': None, 'endTime': None, 'batRemain': None},
        'landAtEndLoc'       : {'startTime': None, 'endTime': None, 'batRemain': None}
    }

    # Initialize status =======================================================
    accTime = 0
    batRemain = batCap

    # Launch from depot =======================================================
    # Takeoff
    timeTakeoffFromStartLoc = DRONE_CRUISE_ALTITUDE / DRONE_TAKEOFF_SPEED
    energyTakeoffFromStartLoc = timeTakeoffFromStartLoc * droneCsmpRate(
        payload = payload,
        vertSpd = DRONE_TAKEOFF_SPEED,
        windSpd = windSpd,
        windDeg = windDeg,
        gndSpd = 0,
        gndDeg = 0)
    accTime += timeTakeoffFromStartLoc
    batRemain -= energyTakeoffFromStartLoc
    if (batRemain <= 0):
        return None
    # Update profile
    profile['launchFromStartLoc']['startTime'] = 0
    profile['launchFromStartLoc']['endTime'] = accTime
    profile['launchFromStartLoc']['batRemain'] = batRemain
    profile['cruiseToCustomer']['startTime'] = accTime

    # Cruise to customer ======================================================
    timeStartLoc2CusLoc = gndDistToCustomer / gndSpdToCus
    energyCruise2Customer = timeStartLoc2CusLoc * droneCsmpRate(
        payload = payload, 
        vertSpd = 0, 
        windSpd = windSpd,
        windDeg = windDeg,
        gndSpd = gndSpdToCus,
        gndDeg = gndDegToCustomer)
    accTime += timeStartLoc2CusLoc
    batRemain -= energyCruise2Customer
    if (batRemain <= 0):
        return None
    # Update profile
    profile['cruiseToCustomer']['endTime'] = accTime
    profile['cruiseToCustomer']['batRemain'] = batRemain
    profile['hoverAtCustomer']['startTime'] = accTime

    # Hover at customer (If hover time > 0) ===================================
    if (hoverTime > 0):
        energyHovering = hoverTime * droneCsmpRate(
            payload = payload,
            vertSpd = 0,
            windSpd = windSpd,
            windDeg = windDeg,
            gndSpd = 0,
            gndDeg = 0)
        accTime += hoverTime
        batRemain -= energyHovering
        if (batRemain <= 0):
            return None
    # Update profile
    profile['hoverAtCustomer']['endTime'] = accTime
    profile['hoverAtCustomer']['batRemain'] = batRemain
    profile['landAtCustomer']['startTime'] = accTime

    # Land at customer ========================================================
    if (landFlag):
        timeLandAtCustomer = DRONE_CRUISE_ALTITUDE / DRONE_LANDING_SPEED
        energyLandAtCustomer = timeLandAtCustomer * droneCsmpRate(
            payload = payload,
            vertSpd = DRONE_LANDING_SPEED,
            windSpd = windSpd,
            windDeg = windDeg,
            gndSpd = 0,
            gndDeg = 0)
        accTime += timeLandAtCustomer
        batRemain -= energyLandAtCustomer
        if (batRemain <= 0):
            return None
    # Update profile
    profile['landAtCustomer']['endTime'] = accTime
    profile['landAtCustomer']['batRemain'] = batRemain
    profile['waitingAtCustomer']['startTime'] = accTime

    # Wait at customer ========================================================
    if (serviceTime > 0):
        accTime += serviceTime
    # Update profile
    profile['waitingAtCustomer']['endTime'] = accTime
    profile['waitingAtCustomer']['batRemain'] = batRemain
    profile['launchFromCustomer']['startTime'] = accTime

    # Launch from customer ====================================================
    if (landFlag):
        timeLaunchFromCustomer = DRONE_CRUISE_ALTITUDE / DRONE_TAKEOFF_SPEED
        energyLaunchFromCustomer = timeLaunchFromCustomer * droneCsmpRate(
            payload = 0,
            vertSpd = DRONE_TAKEOFF_SPEED,
            windSpd = windSpd,
            windDeg = windDeg,
            gndSpd = 0,
            gndDeg = 0)
        accTime += timeLaunchFromCustomer
        batRemain -= energyLaunchFromCustomer
        if (batRemain <= 0):
            return None
    # Update profile
    profile['launchFromCustomer']['endTime'] = accTime
    profile['launchFromCustomer']['batRemain'] = batRemain
    profile['cruiseToEndLoc']['startTime'] = accTime

    # Cruise to depot ======================================================
    timeCustomer2EndLoc = gndDistToEndLoc / gndSpdToCus
    energyCruise2EndLoc = timeCustomer2EndLoc * droneCsmpRate(
        payload = payload, 
        vertSpd = 0, 
        windSpd = windSpd,
        windDeg = windDeg,
        gndSpd = gndSpdFromCus,
        gndDeg = gndDegToEndLoc)
    accTime += timeCustomer2EndLoc
    batRemain -= energyCruise2EndLoc
    if (batRemain <= 0):
        return None

    # Update profile
    profile['cruiseToEndLoc']['endTime'] = accTime
    profile['cruiseToEndLoc']['batRemain'] = batRemain
    profile['landAtEndLoc']['startTime'] = accTime

    # Land at depot ========================================================
    # Land at depot
    timeLandAtEndLoc = DRONE_CRUISE_ALTITUDE / DRONE_LANDING_SPEED
    energyLandAtEndLoc = timeLandAtEndLoc * droneCsmpRate(
        payload = 0,
        vertSpd = DRONE_LANDING_SPEED,
        windSpd = windSpd,
        windDeg = windDeg,
        gndSpd = 0,
        gndDeg = 0)
    accTime += timeLandAtEndLoc
    batRemain -= energyLandAtEndLoc
    if (batRemain <= 0):
        return None
    # Update profile
    profile['landAtEndLoc']['endTime'] = accTime
    profile['landAtEndLoc']['batRemain'] = batRemain

    return accTime

def radarTransit():
    return

def radarTurnaround(
    startLoc = pt,
    windSpd: float = 0,
    windDeg: float = 0,
    batRemain: float = DRONE_BATTARY_CAPACITY,
    hoverTime: float = 0,
    landFlag: bool = DRONE_LANDING_FLAG,
    maxGndSpd: float = DRONE_MAX_GND_SPEED,
    payload: float = DRONE_PARCEL_WEIGHT,
    lod: int = 30) -> dict:

    # Initialize ==============================================================
    # If drone is flying at its maximum ground speed
    maxSpeedRange = []
    # If drone is flying at its maximum endurance
    maxEndurRange = []
    # Poly ====================================================================
    for d in range(lod):
        maxSpeedRange.append(rangeTurnaroundMaxSpd(
            windSpd = windSpd,
            windDeg = windDeg,
            batRemain = batRemain,
            hoverTime = hoverTime,
            landFlag = landFlag,
            maxGndSpd = maxGndSpd,
            payload = payload,
            gndDeg = d * (360 / lod))['gndDist'])
        maxEndurRange.append(rangeTurnaroundEndurance(
            windSpd = windSpd,
            windDeg = windDeg,
            batRemain = batRemain,
            hoverTime = hoverTime,
            landFlag = landFlag,
            maxGndSpd = maxGndSpd,
            payload = payload,
            gndDeg = d * (360 / lod))['gndDist'])

    return {
        'maxSpeedRange': maxSpeedRange,
        'maxEndurRange': maxEndurRange
    }  

def rangeTurnaroundMaxSpd(
    windSpd: float = 0,
    windDeg: float = 0,
    gndDeg: float = 0,
    batRemain: float = DRONE_BATTARY_CAPACITY,
    hoverTime: float = 0,
    landFlag: bool = DRONE_LANDING_FLAG,
    maxGndSpd: float = DRONE_MAX_GND_SPEED,
    payload: float = DRONE_PARCEL_WEIGHT) -> dict:

    # NOTE: 无人机始终以最大可能的速度飞行，可以到达的范围，以及对应的往返剖面

    # Initialize ==============================================================
    mustEnergy = _calEnergyExceptCruise(
        windSpd = windSpd,
        windDeg = windDeg,
        hoverTime = hoverTime,
        landFlag = landFlag,
        payload = payload)
    remainBattery = batRemain - mustEnergy
    if (remainBattery <= 0):
        return None

    # Max speed consumption rate
    loadedMaxSpeedConsumptionRate = droneCsmpRate(
        payload = payload,
        vertSpd = 0,
        windSpd = windSpd,
        windDeg = windDeg,
        gndSpd = maxGndSpd,
        gndDeg = deg)
    noLoadMaxSpeedConsumptionRate = droneCsmpRate(
        payload = 0,
        vertSpd = 0,
        windSpd = windSpd,
        windDeg = windDeg,
        gndSpd = maxGndSpd,
        gndDeg = deg + 180)

    # Assuming maximum ground speed, calculate the maximum endurance
    maxOnewayTime = remainBattery / (loadedMaxSpeedConsumptionRate + noLoadMaxSpeedConsumptionRate)
    r = maxOnewayTime * maxGndSpd

    return {
        'gndDist': r,
        'toCusSpd': maxGndSpd,
        'fromCusSpd': maxGndSpd
    } 

def rangeTurnaroundEndurance(
    windSpd: float = 0,
    windDeg: float = 0,
    gndDeg: float = 0,
    batRemain: float = DRONE_BATTARY_CAPACITY,
    hoverTime: float = 0,
    landFlag: bool = DRONE_LANDING_FLAG,
    maxGndSpd: float = DRONE_MAX_GND_SPEED,
    payload: float = DRONE_PARCEL_WEIGHT) -> dict:

    # Initialize ==============================================================
    mustEnergy = _calEnergyExceptCruise(
        windSpd = windSpd,
        windDeg = windDeg,
        hoverTime = hoverTime,
        landFlag = landFlag,
        payload = payload)
    remainBattery = batRemain - mustEnergy
    if (remainBattery <= 0):
        return None

    # Get maxEndurRange ========================================================
    # Search on the percentage of allocating energy on Go/back
    pLB = 0
    pUB = 1
    p = 0.5
    r = 0
    goGndSpd = None
    backGndSpd = None
    while (pUB - pLB > 0.001):
        go = rangeCruiseEndurance(
            payload = payload,
            cruiseEnergy = (1 - p) * cruiseEnergy,
            windSpd = windSpd,
            windDeg = windDeg,
            gndDeg = gndDeg)
        goMaxEndur = go['gndDist']
        goGndSpd = go['gndSpd']

        back = rangeCruiseEndurance(
            payload = 0,
            cruiseEnergy = p * cruiseEnergy,
            windSpd = windSpd,
            windDeg = windDeg,
            gndDeg = gndDeg + 180)
        backMaxEndur = back['gndDist']
        backGndSpd = back['gndSpd']

        if (goMaxEndur > backMaxEndur):
            pLB = p               
        else:
            pUB = p
        p = pLB + (pUB - pLB) / 2
        r = min(goMaxEndur, backMaxEndur)

    return {
        'gndDist': r,
        'toCusSpd': goGndSpd,
        'fromCusSpd': backGndSpd
    } 

def maxCruiseSpdInDist(
    windSpd: float = 0,
    windDeg: float = 0,
    gndDeg: float = 0,
    gndDist: float = 0,
    payload: float = 0,
    cruiseEnergy: float = DRONE_BATTARY_CAPACITY) -> dict:

    # NOTE: 给定距离方位，最快能飞快送达
    # 先计算经济航速，这是确保一定能到达的航速，然后尽可能更快
    endr = rangeCruiseEndurance(
        windSpd = windSpd,
        windDeg = windDeg,
        gndDeg = gndDeg,
        payload = payload,
        cruiseEnergy = cruiseEnergy)
    if (endr['gndDist'] < gndDist):
        return None

    # Left 永远得是feasible的
    lower = endr['gndSpd']
    upper = DRONE_MAX_GND_SPEED
    lowerConsumption = droneCsmpRate(
        payload = payload,
        vertSpd = 0,
        windSpd = windSpd,
        windDeg = windDeg,
        gndSpd = lower,
        gndDeg = gndDeg)
    upperConsumption = droneCsmpRate(
        payload = payload,
        vertSpd = 0,
        windSpd = windSpd,
        windDeg = windDeg,
        gndSpd = upper,
        gndDeg = gndDeg)
    lowerEnergy = (gndDist / lower) * lowerConsumption
    upperEnergy = (gndDist / upper) * upperConsumption

    mid = None
    while (upper - lower > 0.01):
        mid = lower + (upper - lower) / 2
        midConsumption = droneCsmpRate(
            payload = payload,
            vertSpd = 0,
            windSpd = windSpd,
            windDeg = windDeg,
            gndSpd = mid,
            gndDeg = gndDeg)
        midEnergy = (gndDist / mid) * midConsumption

        if (midEnergy > cruiseEnergy):
            lower = mid
        else:
            upper = mid

    return {
        'maxGndSpd': lower,
        'minGndTime': (gndDist / lower)
    }


    # Calculate energy needed for taking off, hovering and landing ============
    mustEnergy = _calEnergyExceptCruise(
        windSpd = windSpd,
        windDeg = windDeg,
        hoverTime = hoverTime,
        landFlag = landFlag,
        payload = payload)
    remainBattery = DRONE_BATTARY_CAPACITY - mustEnergy
    if (remainBattery <= 0):
        return None

def radarCuriseXY(
    startLoc: pt,
    windSpd: float = 0,
    windDeg: float = 0,
    cruiseEnergy: float = DRONE_BATTARY_CAPACITY,
    maxGndSpd: float = DRONE_MAX_GND_SPEED,
    payload: float = DRONE_PARCEL_WEIGHT,
    lod: int = 30) -> dict:

    # Initialize ==============================================================
    # If drone is flying at its maximum ground speed
    maxSpeedRadar = []
    # If drone is flying at its maximum endurance
    maxEndurRadar = []
    # Poly ====================================================================
    for d in range(lod):
        gndSpeedRadar = rangeCuriseMaxSpd(
            windSpd = windSpd,
            windDeg = windDeg,
            cruiseEnergy = cruiseEnergy,
            maxGndSpd = maxGndSpd,
            payload = payload,
            gndDeg = d * (360 / lod))['gndDist']
        ptSpeedRadar = vrpSolver.ptInDistXY(
            pt = startLoc,
            direction = d * (360 / lod),
            dist = gndSpeedRadar)
        maxSpeedRadar.append(ptSpeedRadar)

        gndEndurRadar = rangeCruiseEndurance(
            windSpd = windSpd,
            windDeg = windDeg,
            cruiseEnergy = cruiseEnergy,
            maxGndSpd = maxGndSpd,
            payload = payload,
            gndDeg = d * (360 / lod))['gndDist']
        ptEndurRadar = vrpSolver.ptInDistXY(
            pt = startLoc,
            direction = d * (360 / lod),
            dist = gndEndurRadar)
        maxEndurRadar.append(ptEndurRadar)

    return {
        'maxSpeedRadar': maxSpeedRadar,
        'maxEndurRadar': maxEndurRadar
    }

# [Done]
def rangeCuriseMaxSpd(
    payload: float = 0,
    cruiseEnergy:  float = DRONE_BATTARY_CAPACITY,
    windSpd: float = 0,
    windDeg: float = 0,
    maxGndSpd: float = DRONE_MAX_GND_SPEED,
    gndDeg: float = 0) -> dict:

    # NOTE: 给定方向，最快能飞多远
    # Max speed consumption rate
    maxSpeedConsumptionRate = droneCsmpRate(
        payload = payload,
        vertSpd = 0,
        windSpd = windSpd,
        windDeg = windDeg,
        gndSpd = maxGndSpd,
        gndDeg = gndDeg)

    # Assuming maximum ground speed, calculate the maximum endurance
    maxDist = (cruiseEnergy / maxSpeedConsumptionRate) * maxGndSpd
    return {
        'gndSpd': maxGndSpd,
        'gndDist': maxDist
    }

# [Done]
def rangeCruiseEndurance(
    payload: float = 0,
    cruiseEnergy:  float = DRONE_BATTARY_CAPACITY,
    windSpd: float = 0,
    windDeg: float = 0,
    maxGndSpd: float = DRONE_MAX_GND_SPEED,
    gndDeg: float = 0) -> dict:

    # NOTE: 给定方向，最远能飞多远

    # bounding of speed =======================================================
    lower = DRONE_MIN_GND_SPEED
    upper = DRONE_MAX_GND_SPEED
    while (upper - lower > 0.01):
        m1 = lower + (upper - lower) / 3
        m2 = upper - (upper - lower) / 3
        m1Consumption = droneCsmpRate(
            payload = payload,
            vertSpd = 0,
            windSpd = windSpd,
            windDeg = windDeg,
            gndSpd = m1,
            gndDeg = gndDeg)
        m2Consumption = droneCsmpRate(
            payload = payload,
            vertSpd = 0,
            windSpd = windSpd,
            windDeg = windDeg,
            gndSpd = m2,
            gndDeg = gndDeg)
        m1Endur = (cruiseEnergy / m1Consumption) * m1
        m2Endur = (cruiseEnergy / m2Consumption) * m2
        if (m1Endur >= m2Endur):
            upper = m2
        else:
            lower = m1        
    savEndur = (cruiseEnergy / lower) * lower

    return {
        'gndSpd': lower,
        'gndDist': savEndur
    }

# [Done]
def droneCsmpRate(
    payload: float = 0,
    windSpd: float = 0,
    windDeg: float = 0,
    vertSpd: float = 0,
    gndSpd: float = 0,
    gndDeg: float = 0) -> float:

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
    airSpd, airDeg = vrpSolver.polarSubtract([gndSpd, gndDeg], [windSpd, windDeg])

    # Induced power, profile power, parasite power ============================
    # Power = induced power + profile power + parasite power
    # Thrust
    T = math.sqrt(
        ((DRONE_EMPTY_WEIGHT + payload) * g - c5 * (airSpd * math.cos(math.radians(alpha)))**2)**2 
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

# [Done]
def _calEnergyExceptCruise(
    windSpd: float = 0,
    windDeg: float = 0,
    hoverTime: float = 0,
    landFlag: float = 0,
    payload: float = 0):

    # NOTE: 去掉起飞，降落等的毛重

    # Calculate energy needed for taking off, hovering and landing ============
    mustEnergy = 0
    # First takeoff
    mustEnergy += (DRONE_CRUISE_ALTITUDE / DRONE_TAKEOFF_SPEED) * droneCsmpRate(
        payload = payload,
        vertSpd = DRONE_TAKEOFF_SPEED,
        windSpd = windSpd,
        windDeg = windDeg,
        gndSpd = 0,
        gndDeg = 0)
    # Hover
    if (hoverTime != None and hoverTime > 0):
        mustEnergy += hoverTime * droneCsmpRate(
            payload = payload,
            vertSpd = 0,
            windSpd = windSpd,
            windDeg = windDeg,
            gndSpd = 0,
            gndDeg = 0)
    # Land/launch at customer
    if (landFlag):
        mustEnergy += (DRONE_CRUISE_ALTITUDE / DRONE_LANDING_SPEED) * droneCsmpRate(
            payload = payload,
            vertSpd = DRONE_LANDING_SPEED,
            windSpd = windSpd,
            windDeg = windDeg,
            gndSpd = 0,
            gndDeg = 0)
        mustEnergy += (DRONE_CRUISE_ALTITUDE / DRONE_TAKEOFF_SPEED) * droneCsmpRate(
            payload = 0,
            vertSpd = DRONE_TAKEOFF_SPEED,
            windSpd = windSpd,
            windDeg = windDeg,
            gndSpd = 0,
            gndDeg = 0)
    # Land at customer
    mustEnergy += (DRONE_CRUISE_ALTITUDE / DRONE_LANDING_SPEED) * droneCsmpRate(
        payload = 0,
        vertSpd = DRONE_LANDING_SPEED,
        windSpd = windSpd,
        windDeg = windDeg,
        gndSpd = 0,
        gndDeg = 0)

    return mustEnergy
