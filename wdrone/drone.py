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

def optSpdPickupDelivery(
    startLoc: pt,
    deliverLoc: pt,
    pickupLoc: pt,
    endLoc: pt,
    windSpd: float = 0,
    windDeg: float = 0,
    payloadDelivery: float = 0,
    payloadPickup: float = 0,
    mapType: str = 'XY',
    crzErgy: float = DRONE_BATTARY_CAPACITY) -> dict:
    return

def optSpdTransit(
    startLoc: pt,
    endLoc: pt,
    cusLoc: pt,
    windSpd: float = 0,
    windDeg: float = 0,
    payload: float = 0,
    mapType: str = 'XY',
    crzErgy: float = DRONE_BATTARY_CAPACITY) -> dict:

    # Ground speedA degree =====================================================
    gndDegToCus, gndDegToEnd, gndDistToCus, gndDistToEnd = None, None, None, None
    if (mapType == 'XY'):
        gndDegToCus = vrpSolver.headingXY(startLoc, cusLoc)
        gndDegToEnd = vrpSolver.headingXY(cusLoc, endLoc)
        gndDistToCus = vrpSolver.distEuclideanXY(startLoc, cusLoc)['dist']
        gndDistToEnd = vrpSolver.distEuclideanXY(cusLoc, endLoc)['dist']
    elif (mapType == 'LatLon'):
        gndDegToCus = vrpSolver.headingLatLon(startLoc, cusLoc)
        gndDegToEnd = vrpSolver.headingLatLon(cusLoc, endLoc)
        gndDistToCus = vrpSolver.distLatLon(startLoc, cusLoc, distUnit='meter')['dist']
        gndDistToEnd = vrpSolver.distLatLon(cusLoc, endLoc, distUnit='meter')['dist']

    # Go and back energy requirement range ====================================
    # 去的能量需求在goMaxEndur到goMaxSpd之间
    goMaxSpd = ergyReqOnewayMaxSpd(
        windSpd = windSpd,
        windDeg = windDeg,
        gndDeg = gndDegToCus,
        gndDist = gndDistToCus,
        payload = payload,
        )
    goMaxSpdErgy = goMaxSpd['ergy']
    goMaxSpdGndSpd = goMaxSpd['gndSpd']

    goMaxEndur = ergyReqOnewayMaxEndur(
        windSpd = windSpd,
        windDeg = windDeg,
        gndDeg = gndDegToCus,
        gndDist = gndDistToCus,
        payload = payload,
        )
    goMaxEndurErgy = goMaxEndur['ergy']
    goMaxEndurGndSpd = goMaxEndur['gndSpd']

    # 回来的能量需求在backMaxEndur到backMaxSpd之间
    backMaxSpd = ergyReqOnewayMaxSpd(
        windSpd = windSpd,
        windDeg = windDeg,
        gndDeg = gndDegToEnd,
        gndDist = gndDistToEnd,
        payload = 0,
        )
    backMaxSpdErgy = backMaxSpd['ergy']
    backMaxSpdGndSpd = backMaxSpd['gndSpd']

    backMaxEndur = ergyReqOnewayMaxEndur(
        windSpd = windSpd,
        windDeg = windDeg,
        gndDeg = gndDegToEnd,
        gndDist = gndDistToEnd,
        payload = 0,
        )
    backMaxEndurErgy = backMaxEndur['ergy']
    backMaxEndurGndSpd = backMaxEndur['gndSpd']

    # Two extreme cases =======================================================
    if (goMaxSpdErgy + backMaxSpdErgy <= crzErgy):
        # 全程最大速度
        return {
            'toCusSpd': goMaxSpdGndSpd,
            'fromCusSpd': backMaxSpdGndSpd,
            'toCusTime': (gndDistToCus / goMaxSpdGndSpd),
            'fromCusTime': (gndDistToEnd / backMaxSpdGndSpd),
            'ergy': goMaxSpdErgy + backMaxSpdErgy
        }
    print(goMaxSpd, goMaxEndur, backMaxSpd, backMaxEndur)
    if (goMaxEndurErgy + backMaxEndurErgy > crzErgy):
        print("NOT REACHABLE")
        return None

    # Get best energy allocation ==============================================
    # Search on the percentage of allocating energy on Go/back
    # 约束包括：
    # 1. go的ergy的范围在goMaxEndur['ergy']到goMaxSpd['ergy']之间
    # 2. back的ergy的范围在backMaxEndur['ergy']到backMaxSpd['ergy']之间
    # 3. go和back的ergy加起来等于crzErgy

    # 计算pLB - 尽可能给更多的ergy到返程，同时去程给最小需要的能量
    goErgy = goMaxEndurErgy
    pLB = goErgy / crzErgy
    pLBBack = optSpdOneway(
        startLoc = cusLoc,
        cusLoc = endLoc,
        windSpd = windSpd,
        windDeg = windDeg,
        payload = 0,
        mapType = mapType,
        crzErgy = (1 - pLB) * crzErgy)
    pLBTime = (gndDistToCus / goMaxEndurGndSpd) + pLBBack['time']

    # 计算pUB - 尽可能给更多的ergy到去程，同时返程给最小需要的能量
    backErgy = backMaxEndurErgy
    goErgy = crzErgy - backErgy
    pUB = goErgy / crzErgy
    pUBGo = optSpdOneway(
        startLoc = cusLoc,
        cusLoc = endLoc,
        windSpd = windSpd,
        windDeg = windDeg,
        payload = payload,
        mapType = mapType,
        crzErgy = pUB * crzErgy)
    pUBTime = pUBGo['time'] + (gndDistToEnd / backMaxSpdGndSpd)

    while (pUB - pLB > 0.001):
        # 计算两个分割点
        p1 = pLB + (pUB - pLB) / 3
        p2 = pUB - (pUB - pLB) / 3

        # 对于p1
        # goErgy = p1 * crzErgy
        # backErgy = (1 - p1) * crzErgy
        # 去程在给定goErgy的情况下最快速度
        p1Go = optSpdOneway(
            startLoc = startLoc,
            cusLoc = cusLoc,
            windSpd = windSpd,
            windDeg = windDeg,
            payload = payload,
            mapType = mapType,
            crzErgy = p1 * crzErgy)
        p1Back = optSpdOneway(
            startLoc = cusLoc,
            cusLoc = endLoc,
            windSpd = windSpd,
            windDeg = windDeg,
            payload = 0,
            mapType = mapType,
            crzErgy = (1 - p1) * crzErgy)
        p1Time = p1Go['time'] + p1Back['time']

        # 对于p2
        # goErgy = p1 * crzErgy
        # backErgy = (1 - p1) * crzErgy
        # 去程在给定goErgy的情况下最快速度
        p2Go = optSpdOneway(
            startLoc = startLoc,
            cusLoc = cusLoc,
            windSpd = windSpd,
            windDeg = windDeg,
            payload = payload,
            mapType = mapType,
            crzErgy = p2 * crzErgy)
        p2Back = optSpdOneway(
            startLoc = cusLoc,
            cusLoc = endLoc,
            windSpd = windSpd,
            windDeg = windDeg,
            payload = 0,
            mapType = mapType,
            crzErgy = (1 - p2) * crzErgy)
        p2Time = p2Go['time'] + p2Back['time']

        if (p1Time <= p2Time):
            pUB = p2
            pUPTime = p2Time
        else:
            pLB = p1
            pLBTime = p1Time

    pBest = pLB + (pUB - pLB) / 2
    pBestGo = optSpdOneway(
        startLoc = startLoc,
        cusLoc = cusLoc,
        windSpd = windSpd,
        windDeg = windDeg,
        payload = payload,
        mapType = mapType,
        crzErgy = pBest * crzErgy)
    pBestBack = optSpdOneway(
        startLoc = cusLoc,
        cusLoc = endLoc,
        windSpd = windSpd,
        windDeg = windDeg,
        payload = 0,
        mapType = mapType,
        crzErgy = (1 - pBest) * crzErgy)

    return {
        'toCusSpd': pBestGo['gndSpd'],
        'fromCusSpd': pBestBack['gndSpd'],
        'toCusTime': pBestGo['time'],
        'fromCusTime': pBestBack['time'],
        'ergy': pBestGo['ergy'] + pBestBack['ergy']
    }

# Turnaround ==================================================================
def optSpdTurnaround(
    startLoc: pt,
    cusLoc: pt,
    windSpd: float = 0,
    windDeg: float = 0,
    payload: float = 0,
    mapType: str = 'XY',
    crzErgy: float = DRONE_BATTARY_CAPACITY) -> dict:
    return optSpdTransit(
        startLoc = startLoc,
        endLoc = startLoc,
        cusLoc = cusLoc,
        windSpd = windSpd,
        windDeg = windDeg,
        payload = payload,
        mapType = 'XY',
        crzErgy = crzErgy)

def ergyReqTurnaroundMaxSpd(
    windSpd: float = 0,
    windDeg: float = 0,
    gndDeg: float = 0,
    gndDist: float = 0,
    payload: float = 0,
    ):

    # 给定方向距离，若以最大速度飞行，需要多少能量    
    goCsmp = droneCsmpRate(
        payload = payload,
        vertSpd = 0,
        windSpd = windSpd,
        windDeg = windDeg,
        gndSpd = DRONE_MAX_GND_SPEED,
        gndDeg = gndDeg)
    goErgy = (gndDist / DRONE_MAX_GND_SPEED) * goCsmp
    backCsmp = droneCsmpRate(
        payload = 0,
        vertSpd = 0,
        windSpd = windSpd,
        windDeg = windDeg,
        gndSpd = DRONE_MAX_GND_SPEED,
        gndDeg = gndDeg + 180)
    backErgy = (gndDist / DRONE_MAX_GND_SPEED) * backCsmp

    ergy = goErgy + backErgy

    return {
        'toCusSpd': DRONE_MAX_GND_SPEED,
        'fromCusSpd': DRONE_MAX_GND_SPEED,
        'ergy': ergy
    }

def ergyReqTurnaroundMaxEndur(
    windSpd: float = 0,
    windDeg: float = 0,
    gndDeg: float = 0,
    gndDist: float = 0,
    payload: float = 0,
    ):

    # 给定方向距离，若以最经济速度飞行，最少需要多少能量
    goErgy = ergyReqOnewayMaxEndur(
        windSpd = windSpd,
        windDeg = windDeg,
        gndDeg = gndDeg,
        gndDist = gndDist,
        payload = payload,
        )
    backErgy = ergyReqOnewayMaxEndur(
        windSpd = windSpd,
        windDeg = windDeg,
        gndDeg = gndDeg + 180,
        gndDist = gndDist,
        payload = 0,
        )

    return {
        'toCusSpd': goErgy['gndSpd'],
        'fromCusSpd': backErgy['gndSpd'],
        'ergy': goErgy['ergy'] + backErgy['ergy']
    }

def radarTurnaround(
    startLoc: pt,
    windSpd: float = 0,
    windDeg: float = 0,
    crzErgy: float = DRONE_BATTARY_CAPACITY,
    payload: float = DRONE_PARCEL_WEIGHT,
    mapType: str = 'XY',
    lod: int = 30) -> dict:

    # Initialize ==============================================================
    # If drone is flying at its maximum ground speed
    maxSpeedRadar = []
    # If drone is flying at its maximum endurance
    maxEndurRadar = []
    # Poly ====================================================================
    for d in range(lod):
        gndSpeedRadar = rangeTurnaroundMaxSpd(
            windSpd = windSpd,
            windDeg = windDeg,
            crzErgy = crzErgy,
            payload = payload,
            gndDeg = d * (360 / lod))['gndDist']
        if (mapType == 'XY'):
            maxSpeedRadar.append(
                vrpSolver.ptInDistXY(
                    pt = startLoc,
                    direction = d * (360 / lod),
                    dist = gndSpeedRadar))
        elif (mapType == 'LatLon'):
            maxSpeedRadar.append(
                vrpSolver.ptInDistLatLon(
                    pt = startLoc,
                    direction = d * (360 / lod),
                    distMeters = gndSpeedRadar))

        gndEndurRadar = rangeTurnaroundMaxEndur(
            windSpd = windSpd,
            windDeg = windDeg,
            crzErgy = crzErgy,
            payload = payload,
            gndDeg = d * (360 / lod))['gndDist']
        if (mapType == 'XY'):
            maxEndurRadar.append(
                vrpSolver.ptInDistXY(
                    pt = startLoc,
                    direction = d * (360 / lod),
                    dist = gndEndurRadar))
        elif (mapType == 'LatLon'):
            maxEndurRadar.append(
                vrpSolver.ptInDistLatLon(
                    pt = startLoc,
                    direction = d * (360 / lod),
                    distMeters = gndEndurRadar))

    return {
        'maxSpeedRadar': maxSpeedRadar,
        'maxEndurRadar': maxEndurRadar
    }

def rangeTurnaroundMaxSpd(
    payload: float = 0,
    crzErgy:  float = DRONE_BATTARY_CAPACITY,
    windSpd: float = 0,
    windDeg: float = 0,
    gndDeg: float = 0) -> dict:

    # NOTE: 无人机始终以最大可能的速度飞行，可以到达的范围
    # Max speed consumption rate
    loadedCsmpRate = droneCsmpRate(
        payload = payload,
        vertSpd = 0,
        windSpd = windSpd,
        windDeg = windDeg,
        gndSpd = DRONE_MAX_GND_SPEED,
        gndDeg = gndDeg)
    emptyCsmpRate = droneCsmpRate(
        payload = 0,
        vertSpd = 0,
        windSpd = windSpd,
        windDeg = windDeg,
        gndSpd = DRONE_MAX_GND_SPEED,
        gndDeg = gndDeg + 180)

    # Assuming maximum ground speed, calculate the maximum endurance
    maxOnewayTime = crzErgy / (loadedCsmpRate + emptyCsmpRate)
    r = maxOnewayTime * DRONE_MAX_GND_SPEED

    return {        
        'toCusSpd': DRONE_MAX_GND_SPEED,
        'fromCusSpd': DRONE_MAX_GND_SPEED,
        'gndDist': r
    } 

def rangeTurnaroundMaxEndur(
    payload: float = 0,
    crzErgy:  float = DRONE_BATTARY_CAPACITY,
    windSpd: float = 0,
    windDeg: float = 0,
    gndDeg: float = 0) -> dict:

    # Get maxEndurRange ========================================================
    # Search on the percentage of allocating energy on Go/back
    pLB = 0
    pUB = 1
    p = 0.5
    r = 0
    goGndSpd = None
    backGndSpd = None
    while (pUB - pLB > 0.001):
        # 用(1 - p) * crzErgy能到达的最大距离
        go = rangeOnewayMaxEndur(
            payload = payload,
            crzErgy = (1 - p) * crzErgy,
            windSpd = windSpd,
            windDeg = windDeg,
            gndDeg = gndDeg)
        goMaxEndur = go['gndDist']
        goGndSpd = go['gndSpd']
        # 用p * crzErgy能返回的最大距离
        back = rangeOnewayMaxEndur(
            payload = 0,
            crzErgy = p * crzErgy,
            windSpd = windSpd,
            windDeg = windDeg,
            gndDeg = gndDeg + 180)
        backMaxEndur = back['gndDist']
        backGndSpd = back['gndSpd']

        # 如果go的这段能飞更远，则匀一点给back
        if (goMaxEndur > backMaxEndur):
            pLB = p               
        else:
            pUB = p
        p = pLB + (pUB - pLB) / 2
        r = min(goMaxEndur, backMaxEndur)

    return {        
        'toCusSpd': goGndSpd,
        'fromCusSpd': backGndSpd,
        'gndDist': r
    } 

# One-way =====================================================================
def optSpdOneway(
    startLoc: pt,
    cusLoc: pt,
    windSpd: float = 0,
    windDeg: float = 0,
    payload: float = 0,
    mapType: str = 'XY',
    crzErgy: float = DRONE_BATTARY_CAPACITY) -> dict:

    gndDeg = None
    gndDist = None
    if (mapType == 'XY'):
        gndDeg = vrpSolver.headingXY(startLoc, cusLoc)
        gndDist = vrpSolver.distEuclideanXY(startLoc, cusLoc)['dist']
    elif (mapType == 'LatLon'):
        gndDeg = vrpSolver.headingLatLon(startLoc, cusLoc)
        gndDist = vrpSolver.distLatLon(startLoc, cusLoc, distUnit='meter')['dist']

    # NOTE: 给定距离方位，最快能飞快送达
    # 先计算经济航速，这是确保一定能到达的航速，然后尽可能更快
    endr = rangeOnewayMaxEndur(
        windSpd = windSpd,
        windDeg = windDeg,
        gndDeg = gndDeg,
        payload = payload,
        crzErgy = crzErgy)
    if (endr['gndDist'] < gndDist):
        return None
    endrSpd = endr['gndSpd']

    # Left 永远得是feasible的
    lower = endrSpd
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

    # 先看看最快速度下电量是不是够用，够用就是最快速度
    if (upperEnergy <= crzErgy):
        return {
            'gndSpd': upper,
            'time': (gndDist / upper),
            'ergy': upperEnergy
        }

    # 最快速度电量不够的话就飞行速度进行二分查找
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

        # 如果中间的速度能飞到，说明还能提速
        if (midEnergy <= crzErgy):
            lower = mid
            lowerEnergy = midEnergy
        # 否则速度往下面的区间搜索
        else:
            upper = mid
            upperEnergy = midEnergy

    return {
        'gndSpd': lower,
        'time': (gndDist / lower),
        'ergy': lowerEnergy
    }

def ergyReqOnewayMaxSpd(
    windSpd: float = 0,
    windDeg: float = 0,
    gndDeg: float = 0,
    gndDist: float = 0,
    payload: float = 0,
    ):

    # 给定方向距离，若以最大速度飞行，需要多少能量    
    ergyCsmp = droneCsmpRate(
        payload = payload,
        vertSpd = 0,
        windSpd = windSpd,
        windDeg = windDeg,
        gndSpd = DRONE_MAX_GND_SPEED,
        gndDeg = gndDeg)

    ergy = (gndDist / DRONE_MAX_GND_SPEED) * ergyCsmp

    return {
        'gndSpd': DRONE_MAX_GND_SPEED,
        'ergy': ergy
    }

def ergyReqOnewayMaxEndur(
    windSpd: float = 0,
    windDeg: float = 0,
    gndDeg: float = 0,
    gndDist: float = 0,
    payload: float = 0,
    ):

    # 给定方向距离，若以最经济速度飞行，最少需要多少能量
    lower = 0
    upper = DRONE_MAX_GND_SPEED
    ergy = None

    sample = None
    ergy = None
    delta = 0.01

    while (upper - lower > 0.01):
        sample = lower + (upper - lower) / 2

        m1 = max(lower, sample - delta)
        m2 = min(sample + delta, upper)

        m1CsmpRate = droneCsmpRate(
            payload = payload,
            vertSpd = 0,
            windSpd = windSpd,
            windDeg = windDeg,
            gndSpd = m1,
            gndDeg = gndDeg)
        m2CsmpRate = droneCsmpRate(
            payload = payload,
            vertSpd = 0,
            windSpd = windSpd,
            windDeg = windDeg,
            gndSpd = m2,
            gndDeg = gndDeg)

        m1Ergy = (gndDist / m1) * m1CsmpRate if m1 != 0 else float('inf')
        m2Ergy = (gndDist / m2) * m2CsmpRate
        
        if (m1Ergy <= m2Ergy):
            upper = sample
        else:
            lower = sample

    sampleCsmpRate = droneCsmpRate(
            payload = payload,
            vertSpd = 0,
            windSpd = windSpd,
            windDeg = windDeg,
            gndSpd = sample,
            gndDeg = gndDeg)
    sampleErgy = (gndDist / sample) * sampleCsmpRate

    return {
        'gndSpd': sample,
        'ergy': sampleErgy
    }

def radarOneway(
    startLoc: pt,
    windSpd: float = 0,
    windDeg: float = 0,
    crzErgy: float = DRONE_BATTARY_CAPACITY,
    payload: float = DRONE_PARCEL_WEIGHT,
    mapType: str = 'XY',
    lod: int = 30) -> dict:

    # Initialize ==============================================================
    # If drone is flying at its maximum ground speed
    maxSpeedRadar = []
    # If drone is flying at its maximum endurance
    maxEndurRadar = []
    # Poly ====================================================================
    for d in range(lod):
        gndSpeedRadar = rangeOnewayMaxSpd(
            windSpd = windSpd,
            windDeg = windDeg,
            crzErgy = crzErgy,
            payload = payload,
            gndDeg = d * (360 / lod))['gndDist']
        if (mapType == 'XY'):
            maxSpeedRadar.append(
                vrpSolver.ptInDistXY(
                    pt = startLoc,
                    direction = d * (360 / lod),
                    dist = gndSpeedRadar))
        elif (mapType == 'LatLon'):
            maxSpeedRadar.append(
                vrpSolver.ptInDistLatLon(
                    pt = startLoc,
                    direction = d * (360 / lod),
                    distMeters = gndSpeedRadar))

        gndEndurRadar = rangeOnewayMaxEndur(
            windSpd = windSpd,
            windDeg = windDeg,
            crzErgy = crzErgy,
            payload = payload,
            gndDeg = d * (360 / lod))['gndDist']
        if (mapType == 'XY'):
            maxEndurRadar.append(
                vrpSolver.ptInDistXY(
                    pt = startLoc,
                    direction = d * (360 / lod),
                    dist = gndEndurRadar))
        elif (mapType == 'LatLon'):
            maxEndurRadar.append(
                vrpSolver.ptInDistLatLon(
                    pt = startLoc,
                    direction = d * (360 / lod),
                    distMeters = gndEndurRadar))

    return {
        'maxSpeedRadar': maxSpeedRadar,
        'maxEndurRadar': maxEndurRadar
    }

def rangeOnewayMaxSpd(
    payload: float = 0,
    crzErgy:  float = DRONE_BATTARY_CAPACITY,
    windSpd: float = 0,
    windDeg: float = 0,
    gndDeg: float = 0) -> dict:

    # NOTE: 给定方向，最快能飞多远
    # Max speed consumption rate
    maxSpdCsmpRate = droneCsmpRate(
        payload = payload,
        vertSpd = 0,
        windSpd = windSpd,
        windDeg = windDeg,
        gndSpd = DRONE_MAX_GND_SPEED,
        gndDeg = gndDeg)

    # Assuming maximum ground speed, calculate the maximum endurance
    maxDist = (crzErgy / maxSpdCsmpRate) * DRONE_MAX_GND_SPEED
    return {
        'gndSpd': DRONE_MAX_GND_SPEED,
        'gndDist': maxDist
    }

def rangeOnewayMaxEndur(
    payload: float = 0,
    crzErgy:  float = DRONE_BATTARY_CAPACITY,
    windSpd: float = 0,
    windDeg: float = 0,
    gndDeg: float = 0) -> dict:

    # NOTE: 给定方向，最远能飞多远
    # bounding of speed =======================================================
    lower = 0
    upper = DRONE_MAX_GND_SPEED
    lowerCsmpRate = droneCsmpRate(
            payload = payload,
            vertSpd = 0,
            windSpd = windSpd,
            windDeg = windDeg,
            gndSpd = lower,
            gndDeg = gndDeg)
    upperCsmpRate = droneCsmpRate(
            payload = payload,
            vertSpd = 0,
            windSpd = windSpd,
            windDeg = windDeg,
            gndSpd = upper,
            gndDeg = gndDeg)
    while (upper - lower > 0.01):
        m1 = lower + (upper - lower) / 3
        m2 = upper - (upper - lower) / 3
        m1CsmpRate = droneCsmpRate(
            payload = payload,
            vertSpd = 0,
            windSpd = windSpd,
            windDeg = windDeg,
            gndSpd = m1,
            gndDeg = gndDeg)
        m2CsmpRate = droneCsmpRate(
            payload = payload,
            vertSpd = 0,
            windSpd = windSpd,
            windDeg = windDeg,
            gndSpd = m2,
            gndDeg = gndDeg)
        m1Endur = (crzErgy / m1CsmpRate) * m1
        m2Endur = (crzErgy / m2CsmpRate) * m2
        if (m1Endur >= m2Endur):
            upper = m2
            upperCsmpRate = m2CsmpRate
        else:
            lower = m1
            lowerCsmpRate = m1CsmpRate
    savEndur = (crzErgy / lowerCsmpRate) * lower

    return {
        'gndSpd': lower,
        'gndDist': savEndur
    }

# Basic functions =============================================================
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

def ergyMustSpend(
    payload: float = 0,
    windSpd: float = 0,
    windDeg: float = 0,
    hoverTime: float = 0,
    landFlag: float = 0):

    # NOTE: 去掉起飞，降落等的毛重
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
    # Land at endLoc
    mustEnergy += (DRONE_CRUISE_ALTITUDE / DRONE_LANDING_SPEED) * droneCsmpRate(
        payload = 0,
        vertSpd = DRONE_LANDING_SPEED,
        windSpd = windSpd,
        windDeg = windDeg,
        gndSpd = 0,
        gndDeg = 0)

    return mustEnergy
