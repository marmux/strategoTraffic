#!/usr/bin/env python
"""
@file    runner.py
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

@modified  Marco Muniz {muniz@cs.aau.dk,marmux@gmail.com}
@date    2017

for research purposes 
Uppaal Stratego for Intelligent Traffic Lights 
c.f. ../../strategoTraffic.pdf


"""
from __future__ import absolute_import
from __future__ import print_function

import os
import sys
import optparse
import subprocess
import random


class Configuration:
    def __init__(self, scenario):
        maxWEtoLeft = 100.
        maxWEtoWE = 850.
        maxWEtoRight = 60.
        maxNStoLeft = 60.
        maxNStoNS = 200.
        maxNStoRight = 60.
        self.scenario = scenario
        self.tcH = {}
        self.routeH = {}
        self.pH = {}
        if scenario == "MAX":
            multiplier = 1.
        if scenario == "MID":
            multiplier = 0.6
        if scenario == "LOW":
            multiplier = 0.3
        self.pH['NSW'] = multiplier * maxNStoLeft / 3600            
        self.pH['NSE'] = multiplier * maxNStoRight / 3600
        self.pH['NSS'] = multiplier * maxNStoNS / 3600
        self.pH['SNE'] = multiplier * maxNStoLeft / 3600
        self.pH['SNW'] = multiplier * maxNStoRight / 3600
        self.pH['SNN'] = multiplier * maxNStoNS / 3600        
        self.pH['WEN'] = multiplier * maxWEtoLeft / 3600
        self.pH['WES'] = multiplier * maxWEtoRight / 3600
        self.pH['WEE'] = multiplier * maxWEtoWE / 3600
        self.pH['EWS'] = multiplier * maxWEtoLeft / 3600
        self.pH['EWN'] = multiplier * maxWEtoRight / 3600
        self.pH['EWW'] = multiplier * maxWEtoWE / 3600
        self.tcH['NSW'] = 0
        self.tcH['NSE'] = 0
        self.tcH['NSS'] = 0
        self.tcH['SNE'] = 0
        self.tcH['SNW'] = 0
        self.tcH['SNN'] = 0
        self.tcH['WEN'] = 0
        self.tcH['WES'] = 0
        self.tcH['WEE'] = 0
        self.tcH['EWS'] = 0
        self.tcH['EWN'] = 0
        self.tcH['EWW'] = 0
        self.routeH['NSW'] = 'up_left'
        self.routeH['NSE'] = 'up_right'
        self.routeH['NSS'] = 'up'
        self.routeH['SNE'] = 'down_right'
        self.routeH['SNW'] = 'down_left'
        self.routeH['SNN'] = 'down'
        self.routeH['WEN'] = 'left_up'
        self.routeH['WES'] = 'left_down'
        self.routeH['WEE'] = 'left'
        self.routeH['EWS'] = 'right_down'
        self.routeH['EWN'] = 'right_up'
        self.routeH['EWW'] = 'right'
        self.directions = ['NSW','NSE','NSS','SNE','SNW','SNN','WEN','WES','WEE','EWS','EWN','EWW']
        self.nrDirections = len(self.directions)
        
            
def generate_routefile(scenario, outputfile):
    random.seed(42)  # make tests reproducible
    N = 7200  # number of time steps
    cf = Configuration(scenario)
    print(cf.pH)
    with open("data/"+outputfile, "w") as routes:
        totalCars = 0;
        print("""<routes>
        <vType id="typeWE" accel="0.8" decel="4.5" sigma="0.5" length="5" minGap="2.5" maxSpeed="50.00" guiShape="passenger"/>
	<vType id="typeNS" accel="0.8" decel="4.5" sigma="0.5" length="5" minGap="2.5" maxSpeed="50.00" guiShape="passenger"/>
	<route id="left" edges="366845096#0 366845096#1 366845096#20 366845096#22 366845096#3 366845096#4 366845096#5"/>
	<route id="left_up" edges="366845096#0 366845096#1 366845096#20 366845096#22 -157076458#1 -157076458#0"/>
	<route id="left_down" edges="366845096#0 366845096#1 366845096#20 366845096#22 -26446721#2 -26446721#1 -26446721#0"/>
	<route id="right" edges="-366845096#5 -366845096#4 -366845096#30 -366845096#32 -366845096#2 -366845096#1 -366845096#0 "/>
	<route id="right_up" edges="-366845096#5 -366845096#4 -366845096#30 -366845096#32 -157076458#1 -157076458#0 "/>
	<route id="right_down" edges="-366845096#5 -366845096#4 -366845096#30 -366845096#32 -26446721#2 -26446721#1 -26446721#0"/>
	<route id="up" edges="157076458#0 157076458#1 -26446721#2 -26446721#1 -26446721#0"/>
	<route id="up_left" edges="157076458#0 157076458#1 -366845096#2 -366845096#1 -366845096#0"/>
        <route id="up_right" edges="157076458#0 157076458#1 366845096#3 366845096#4 366845096#5"/>
	<route id="down" edges="26446721#0 26446721#1 26446721#2 -157076458#1 -157076458#0"/>
        <route id="down_left" edges="26446721#0 26446721#1 26446721#2 -366845096#2 -366845096#1 -366845096#0"/>
	<route id="down_right" edges="26446721#0 26446721#1 26446721#2 366845096#3 366845096#4 366845096#5"/>""", file=routes)
        lastVeh = 0
        vehNr = 0
        for i in range(N):
            for j in range(cf.nrDirections):
                d = cf.directions[j]                
                if random.uniform(0, 1) < cf.pH[d]: 
                    route = cf.routeH[d]
                    cf.tcH[d] = cf.tcH[d] + 1
                    print('<vehicle id="%s_%i" type="typeWE" route="%s" depart="%i" />' % (
                        route, vehNr, route, i), file=routes)
                    vehNr += 1
                    lastVeh = i
        print('<!-- totalCars="%i -->' % (vehNr), file=routes)
        print('<!-- CarsPerDirection="%s -->' % (str(cf.tcH)), file=routes)        
        print("</routes>", file=routes)
        print(cf.tcH)

def get_options():
    optParser = optparse.OptionParser()
    optParser.add_option("--scenario", type="string", dest="scenario",default="MAX")
    optParser.add_option("--output", type="string", dest="output",default="nylandsvejPlain.rou.xml")
    options, args = optParser.parse_args()
    return options

# this is the main entry point of this script
if __name__ == "__main__":
    options = get_options()
    generate_routefile(options.scenario,options.output)
