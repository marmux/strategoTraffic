#!/usr/bin/env python

"""@file    runner.py
@author  Lena Kalleske
@author  Daniel Krajzewicz
@author  Michael Behrisch
@author  Jakob Erdmann
@date    2009-03-26
@version $Id: runner.py 20433 2016-04-13 08:00:14Z behrisch $

Tutorial for traffic light control via the TraCI interface.

SUMO, Simulation of Urban MObility; see http://sumo.dlr.de/
Copyright (C) 2009-2016 DLR/TS, Germany

This file is part of SUMO.
SUMO is free software; you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation; either version 3 of the License, or
(at your option) any later version.


This file has been modified by Marco Muniz
muniz@cs.aau.dk,marmux@gmail.com for research purposes
Uppaal Stratego for Intelligent Traffic Lights 
c.f. ../../strategoTraffic.pdf

"""

from __future__ import absolute_import
from __future__ import print_function
from callStratego import cStratego

import os
import sys
import optparse
import subprocess
import random
import time

# we need to import python modules from the $SUMO_HOME/tools directory
try:
    sys.path.append(os.path.join(os.path.dirname(
        __file__), '..', '..', '..', '..', "tools"))  # tutorial in tests
    sys.path.append(os.path.join(os.environ.get("SUMO_HOME", os.path.join(
        os.path.dirname(__file__), "..", "..", "..")), "tools"))  # tutorial in docs
    from sumolib import checkBinary
except ImportError:
    sys.exit(
        "please declare environment variable 'SUMO_HOME' as the root directory of your sumo installation (it should contain folders 'bin', 'tools' and 'docs')")

import traci
# the port used for communicating with your sumo instance


rootDir = os.path.abspath(os.getcwd())
pathToResults = rootDir+"/results/"
pathToModels = rootDir+"/models/"
phaseWE = 0
phaseToNS = 1 # from here a transition to NS start
phaseNS = 3
phaseToEW = 4
    # <!-- this is the tl of interest -->
    # <!-- the lanes match the following order ABCDEFGHIJKLMN: -->
    # <!-- A up_left -->
    # <!-- B up_down -->
    # <!-- C up_right -->    
    # <!-- D right_up -->    
    # <!-- E right_left -->    
    # <!-- F right_down low priority -->    
    # <!-- G down_right -->    
    # <!-- H down_up -->
    # <!-- I down_left -->    
    # <!-- J left_down -->
    # <!-- K left_right -->
    # <!-- L left_up low priority -->

    # <!-- encoding program nr 8 -->
    # <tlLogic id="1693132977" type="static" programID="reserve" offset="0">
    #   <phase duration="35" state="rrrGGgrrrGGg"/>
    #   <phase duration="4" state="rrryyyrrryyy"/>
    #   <phase duration="4" state="rrrrrrrrrrrr"/>
    #   <phase duration="20" state="GGgrrrGGgrrr"/>
    #   <phase duration="4" state="yyyrrryyyrrr"/>
    #   <phase duration="4" state="rrrrrrrrrrrr"/>
    # </tlLogic>



def extend(ext, bound, extTime):
    ext = ext + extTime
    return ext

def extend2(phaseTime, maxGreenNS, extTime):    
    return extTime


def run(options):
    """execute the TraCI control loop"""
    print("starting run")
    traci.init(options.port)
    step = 0
    numDetectors = 8
    yellow = 8
    carsPassed = [0] * numDetectors
    carsJammed = [0] * numDetectors
    carsJammedMeters = [0] * numDetectors
    carsPassinge1 = [0] * numDetectors
    carsPassinge2 = [0] * numDetectors
    meanSpeed = [0] * numDetectors
    # indDet = ["1e1","2e1","3e1","4e1"]

    ic_numLoopDet = 6
    ic_numAreDet = 2
    ic_minGreen = 8.0
    #detec for loop induction controller
    ic_loopDet = ["left_e11","left_e12","right_e11","right_e12","up_e11","down_e11"]
    ic_areDet = ["up_e21","down_e21"]
    ic_infLoopDet = [0] * ic_numLoopDet
    ic_infAreDet = [0] * ic_numAreDet
    phaseTime = ic_minGreen
    timeInGreenRest = 0
    
    # ar(A1_0,1) (A1_1,2) (A2_0,3) (A2_1,4) (B1,5) (B2,6)
    areDet = ["left_220", "left_221", "left_22", "right_320", "right_321", "right_30", "down_20", "up_10"]

    # meassuring performance per leg in intersection
    # left A1 down B1
    legs = ["A1","A2","B1","B2"]
    detLegH = {}
    detLegH["A1"] = [ "left_220", "left_221", "left_22", "ex_left_1", "ex_left_2"]
    detLegH["A2"] = [ "right_320", "right_321", "right_30", "ex_right_1", "ex_right_2", "ex_right_3"]
    detLegH["B1"] = [ "down_20", "ex_down_1"]
    detLegH["B2"] = [ "up_10", "ex_up_1"] 
    jamMetLegH = {}
    jamMetLegH["A1"] = 0.0
    jamMetLegH["A2"] = 0.0
    jamMetLegH["B1"] = 0.0
    jamMetLegH["B2"] = 0.0
    jamCarLegH = {}
    jamCarLegH["A1"] = 0
    jamCarLegH["A2"] = 0
    jamCarLegH["B1"] = 0
    jamCarLegH["B2"] = 0
    
    totalJam = 0
    totalJamMeters = 0
    strategoMasterModel = pathToModels + "lowActivityMiniPro.xml"
    strategoMasterModelGreen = pathToModels + "highActivityPro.xml"
    strategoQuery = pathToModels + "query.q"
    strategoLearningMet = "3"
    strategoSuccRuns = "50"
    strategoGoodRuns = "50"
    strategoMaxRuns = "100"
    strategoEvalRuns = "10"
    strategoMaxIterations = "200"
    # we start with phase 1 where EW has green
    phase = phaseWE
    duration = yellow #phase duration from cross.net.xml   
    totaltimeNS = 0
    totaltimeEW = 0
    nextPhase = phaseNS
    strategoRunTime = 4
    phaseTimer = yellow
    strategoTimer = phaseTimer - strategoRunTime
    strategoMaxGreen = 120 #max time in green in one direction
    strategoGreenTimer = 0
    inYellow = True
    idTL = "1693132977"
    print("phase: " + str(phase))
    traci.trafficlights.setProgram(idTL, options.load)
    traci.trafficlights.setPhase(idTL, phase)
    minGreen = 10
    maxGreenEW,maxGreenNS = get_max_green(options)
    extTime = 3
    ext = 0
    timeInPhase = 0
    print("Starting simulation expid=" + str(options.expid))
    count_time = False
    additionalInfoFile = pathToResults+"additionalInfo"+str(options.expid)+".addcsv"
    fout = open(additionalInfoFile, "w")
    add_additional_info_header(fout,legs)
    
    while traci.simulation.getMinExpectedNumber() > 0:
        print(">>>simulation step: " + str(step))
        #carsPassinge2 and carsJammed are used by stratego, they provide partial information        
        carsPassinge2 = get_det_func(traci.areal.getLastStepVehicleNumber,areDet)
        carsJammed = get_det_func(traci.areal.getJamLengthVehicle,areDet)
        carsJammedMeters = get_det_func(traci.areal.getJamLengthMeters,areDet)
        meanSpeed = get_det_func(traci.areal.getLastStepMeanSpeed,areDet)
        jamCarLegH, jamMetLegH = get_messurements(legs, detLegH, jamCarLegH, jamMetLegH,
                                                    traci.areal.getJamLengthVehicle,
                                                    traci.areal.getJamLengthMeters)

        # print_dets_state("carsPassing",areDet,carsPassinge2)
        if options.controller == "loopController":
            if phase == phaseNS or phase == phaseWE: #we controll the green time
                traci.trafficlights.setPhaseDuration(idTL,5000)
            ic_infLoopDet = get_det_func(traci.inductionloop.getLastStepVehicleNumber,ic_loopDet)
            ic_infAreDet = get_det_func(traci.areal.getLastStepVehicleNumber,ic_areDet)
            if phase == phaseNS: #Green in B direction
                if timeInPhase >= maxGreenNS or phaseTime <= 0:
                    #Goto yellow
                    traci.trafficlights.setPhase(idTL, phaseToEW)
                    phase = phaseToEW
                    nextPhase = phaseWE
                    timeInPhase = 0
                else: #Check if we shoud extend the phase timer
                    phaseTime -= 1
                    if ic_infLoopDet[4] >= 1 or ic_infLoopDet[5] >= 1:
                        phaseTime = extend(phaseTime, maxGreenNS, 4.0)
                    cars_on_AreDet = sum(ic_infAreDet) > 0
                    if cars_on_AreDet: #Make sure the time ticks while we stay in one phase
                        phaseTime += 1
                timeInPhase += 1
            elif phase == phaseWE: #Green in A direction
                if timeInPhase >= maxGreenEW or phaseTime <= 0:
                    #Goto yellow
                    traci.trafficlights.setPhase(idTL, phaseToNS)
                    phase = phaseToNS
                    nextPhase = phaseNS
                    timeInPhase = 0
                else:
                    cars_on_AreDet = sum(ic_infAreDet) > 0
                    if ic_infLoopDet[4] == 1 or ic_infLoopDet[5] == 1 or cars_on_AreDet:
                        count_time = True
                    if count_time: #This phase is the resting phase
                        if ic_infLoopDet[0] == 1 or  ic_infLoopDet[4] == 1:
                            phaseTime = extend(phaseTime, maxGreenEW, 4.0)
                        if ic_infLoopDet[1] == 1 or  ic_infLoopDet[2] == 1:
                            phaseTime = extend(phaseTime, maxGreenEW, 4.0)
                        timeInPhase += 1
                        phaseTime -= 1
            else: #In yellow
                if timeInPhase == yellow:
                    #At the end of yellow the timeing variables are reset, the min green time is 10
                    timeInPhase = 0
                    phaseTime = ic_minGreen
                    phase = nextPhase
                    traci.trafficlights.setPhase(idTL, phase)
                    #traci.trafficlights.setPhaseDuration(idTL, 60)
                    count_time = False
                else:
                    timeInPhase += 1
            if options.debug:
                print("Using the loop controller")
                print("Phase in loop controller: ", phase)
                print_dets_state("ic_infLoopDet",ic_loopDet,ic_infLoopDet)
                print_dets_state("ic_infAreDet",ic_areDet,ic_infAreDet)
                print("timeInPhase:" + str(timeInPhase))
                print("phaseTime:" +str(phaseTime))
                print(phase)



        if options.controller == "loopController2":            
            if phase == phaseNS or phase == phaseWE: #we controll the green time
                traci.trafficlights.setPhaseDuration(idTL,5000)
            if options.debug:
                print("Phase in loop controller: ", phase)
            ic_infLoopDet = get_det_func(traci.inductionloop.getLastStepVehicleNumber,ic_loopDet)
            ic_infAreDet = get_det_func(traci.areal.getLastStepVehicleNumber,ic_areDet)
            if phase == phaseNS: #Green in B direction
                if timeInPhase >= maxGreenNS or (phaseTime <= 0 and timeInPhase >= ic_minGreen):
                    #Goto yellow
                    traci.trafficlights.setPhase(idTL, phaseToEW)
                    phase = phaseToEW
                    nextPhase = phaseWE
                    timeInPhase = 0
                else: #Check if we shoud extend the phase timer
                    phaseTime -= 1
                    cars_on_AreDet = sum(ic_infAreDet) > 0
                    if cars_on_AreDet: #Make sure the time ticks while we stay in one phase
                        phaseTime = max([phaseTime, 1])
                    if ic_infLoopDet[4] >= 1 or ic_infLoopDet[5] >= 1: #It is important extent with the largest vaule last
                        phaseTime = max([phaseTime, 4.0])
                timeInPhase += 1
            elif phase == phaseWE: #Green in A direction
                if timeInPhase >= maxGreenEW or (phaseTime <= 0 and timeInGreenRest >= ic_minGreen and count_time):
                    #Goto yellow
                    traci.trafficlights.setPhase(idTL, phaseToNS)
                    phase = phaseToNS
                    nextPhase = phaseNS
                    timeInPhase = 0
                else:
                    phaseTime -= 1
                    cars_on_AreDet = sum(ic_infAreDet) > 0
                    if (ic_infLoopDet[4] == 1 or ic_infLoopDet[5] == 1 or cars_on_AreDet) and not count_time:
                        count_time = True
                    if count_time: #This phase is the resting phase
                        timeInPhase += 1
                    if ic_infLoopDet[0] == 1 or ic_infLoopDet[1] == 1 or ic_infLoopDet[2] == 1 or ic_infLoopDet[3] == 1:
                        #this should extend 2 times if the two loops are in the same lane or one extention per indLoop hit
                        phaseTime = max([phaseTime, 4.0])
                    timeInGreenRest += 1
            else: #In yellow
                if timeInPhase == yellow:
                    #At the end of yellow the timeing variables are reset, the min green time is 10
                    timeInPhase = 0
                    count_time = False
                    timeInGreenRest = 0
                    phaseTime = ic_minGreen
                    phase = nextPhase
                    traci.trafficlights.setPhase(idTL, phase)
                    #traci.trafficlights.setPhaseDuration(idTL, 60)

                else:
                    timeInPhase += 1
            if options.debug:
                print(ic_infLoopDet)
                print(ic_infAreDet)
                print(timeInPhase)
                print(phaseTime)
                print(phase)

                


                
        if options.controller == "stratego":
            if strategoTimer == 0:
                if inYellow:
                    nextPhase,_,_ = cStratego(strategoMasterModel,strategoQuery,
                                              strategoLearningMet,strategoSuccRuns,
                                              strategoMaxRuns,strategoGoodRuns,
                                              strategoEvalRuns,strategoMaxIterations,
                                              options.expid,carsPassinge2, carsJammed,
                                              phase,duration,step,options)
                    duration = 10
                    inYellow = False
                    strategoGreenTimer = 0
                else:
                    nextPhase,_,_ =  cStratego(strategoMasterModelGreen,strategoQuery,
                                               strategoLearningMet,strategoSuccRuns,
                                               strategoMaxRuns,strategoGoodRuns,
                                               strategoEvalRuns,strategoMaxIterations,
                                               options.expid,carsPassinge2, carsJammed,
                                               phase,duration,step,options,greenModel=True,
                                               greenTimer=strategoGreenTimer)
                    if nextPhase == phase:
                        duration = 5
                    else:
                        nextPhase = phaseToNS
                        duration = yellow              
                if options.debug:
                    print("calling stratego \n  strategoTimer:" + str(strategoTimer) + \
                              ", currentPhase:"+ str(phase) + ", nextPhase:" \
                              + str(nextPhase) + ", duration:" + str(duration) + "\n" )
            if phaseTimer == 0:
                phase = nextPhase
                traci.trafficlights.setPhase(idTL,phase)
                totaltimeNS,totaltimeEW = sumtimes(totaltimeNS,totaltimeEW,phase,duration)
                print("setting phase:" + str(phase) + " with duration:" + str(duration))
                if phase == phaseToNS or phase == phaseToEW:
                    inYellow = True
                else:
                    inYellow = False
                if not inYellow:
                    traci.trafficlights.setPhaseDuration(idTL,duration)
                strategoTimer = duration - strategoRunTime
                phaseTimer = duration
            if options.debug:
                print("phase:" + str(phase) + ", nextphase:"+ str(nextPhase) + \
                      ", duration:" + str(duration)  + \
                      ", inYellow:" + str(inYellow) + \
                      ", strategoTimer:" + str(strategoTimer) + \
                      ", strategoGreenTimer:" + str(strategoGreenTimer) + \
                      ", phaseTimer:" + str(phaseTimer))
                print_dets_state("carsPassing",areDet,carsPassinge2)
                print_dets_state("carsJammed",areDet,carsJammed)

            strategoGreenTimer = strategoGreenTimer + 1   
            strategoTimer = strategoTimer - 1
            phaseTimer = phaseTimer - 1

        add_additional_info(fout,step,legs,detLegH,traci.areal.getJamLengthMeters,
                                traci.trafficlights.getPhase(idTL))
        traci.simulationStep()
        step += 1
    print("total carsJammed: " + str(totalJam))    
    traci.close()
    sys.stdout.flush()
    save_results(options.expid,options.controller,options.sumocfg,
                     step, legs, jamCarLegH, jamMetLegH)
    fout.close()

def get_max_green(options):
    if options.load == 'max':
        return 64,40
    if options.load == 'mid':
        return 54,26
    if options.load == 'low':
        return 36,20
    if options.load == 'reserve':
        return 35,20
                    
def sumtimes(totaltimeNS,totaltimeEW,phase,duration):
    ttNS = 0
    ttWE = 0
    if phase == phaseNS:
        ttNS = totaltimeNS + duration
    if phase == phaseWE:
        ttWE = totaltimeEW + duration
    return ttNS, ttWE

def add_additional_info_header(filename,legs):
    csvdata = "step,tlState"
    for i in range(0,len(legs)):
        csvdata = csvdata + "," + legs[i]
    csvdata = csvdata + "\n"    
    filename.write(csvdata)

def add_additional_info(filename,step,legs,detLegH,funJamMet,tlState):
    numLegs = len(legs)
    csvdata = str(step) + "," + str(tlState)
    for i in range(0,numLegs):
        dets_leg_i = detLegH[legs[i]]
        num_dets = len(dets_leg_i)
        legJamStep = 0.0
        for j in range(0,num_dets):
             legJamStep += funJamMet(dets_leg_i[j])
        csvdata = csvdata + "," + str(legJamStep)
    csvdata = csvdata + "\n"
    filename.write(csvdata)

    
def get_messurements(legs, detLegH, jamCarLegH, jamMetLegH, funJamCar, funJamMet):
    numLegs = len(legs)
    for i in range(0,numLegs):
        dets_leg_i = detLegH[legs[i]]
        num_dets = len(dets_leg_i)
        for j in range(0,num_dets):
            jamCarLegH[legs[i]] = jamCarLegH[legs[i]] + funJamCar(dets_leg_i[j])
            jamMetLegH[legs[i]] = jamMetLegH[legs[i]] + funJamMet(dets_leg_i[j])
    return jamCarLegH, jamMetLegH
    

def save_results(expid,controller,scenario,totalSimTime, legs, jamCarLegH, jamMetLegH):
    numLegs = len(legs)
    totalJamMeters = 0.0
    totalJam = 0
    strJamMetLeg = ""
    strJamCarLeg = ""
    for i in range(0,numLegs):
        totalJamMeters = totalJamMeters + jamMetLegH[legs[i]]
        strJamMetLeg = strJamMetLeg + "," + str(jamMetLegH[legs[i]])        
        totalJam = totalJam + jamCarLegH[legs[i]]
        strJamCarLeg = strJamCarLeg + "," + str(jamCarLegH[legs[i]])
    
    filename = pathToResults+"exp"+str(expid)+".csv"
    csvdata = str(expid) \
      + "," + controller \
      + "," + scenario  \
      + "," + str(totalJamMeters) \
      + "," + str(totalJam) \
      + "," + str(totalSimTime) \
      + strJamMetLeg + strJamCarLeg \
      + "\n"
    with open(filename, 'w') as f:        
        f.write(csvdata)
        f.close()
    
def print_dets_state(msg,dets,res):
    print(msg + " detectors: " +str(dets) + " values: " + str(res))

def countJam(carsJammed):
    numDet = len(carsJammed) 
    res = 0
    for i in range(0,numDet):
        res = res + carsJammed[i]
    return res

def get_det_func(func,dets):
    numDet = len(dets)
    res = [0] * numDet
    for deti in range(0,numDet):
        res[deti] = func(dets[deti])
    return res   

def debug_print(options, msg):
    if options.debug:
        print(msg)
        
def get_options():
    optParser = optparse.OptionParser()
    optParser.add_option("--nogui", action="store_true",
                         default=False, help="run the commandline version of sumo")
    optParser.add_option("--debug", action="store_true",
                         default=False, help="display debug information")
    optParser.add_option("--port", type="int", dest="port",default=8873)
    optParser.add_option("--expid", type="int", dest="expid")
    optParser.add_option("--sumocfg", type="string", dest="sumocfg",
                             default="data/nylandsvejPlain.sumocfg")
    optParser.add_option("--load", type="string", dest="load",default="reserve")
    optParser.add_option("--controller", type="string", dest="controller",default="static")    
    options, args = optParser.parse_args()
    return options


                  
# this is the main entry point of this script
if __name__ == "__main__":
    options = get_options()
    print("im am here: " + os.getcwd())
    if options.nogui:
        sumoBinary = checkBinary('sumo')
    else:
        sumoBinary = checkBinary('sumo-gui')
    # this is the normal way of using traci. sumo is started as a
    # subprocess and then the python script connects and runs
    tripinfofile = "results/tripinfo" + str(options.expid) + ".xml"
    sumoProcess = subprocess.Popen([sumoBinary, "-c", options.sumocfg, "--tripinfo-output",
                                    tripinfofile, "--remote-port", str(options.port)], stdout=sys.stdout,
                                   stderr=sys.stderr)
    run(options)
    sumoProcess.wait()
