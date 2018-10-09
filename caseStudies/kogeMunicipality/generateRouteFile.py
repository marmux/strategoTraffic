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
PORT = 8873

class Configuration:
    def __init__(self, scenario):
        if scenario == "LC_ALL":
            self.pWE = 1. / 4 
            self.pEW = 1. / 4
            self.pNS = 1. / 8
            self.pNSW = 1. / 3
            self.pSN = 1. / 8
            self.pSNE = 1. / 3
            self.pWEN = 1. / 4
            self.pWES = 1. / 4
            self.pEWS = 1. / 4
            self.pEWN = 1. / 4
        if scenario == "LC_NS_FC_WE":
            self.pWE = 1. / 16 
            self.pEW = 1. / 16
            self.pNS = 1. / 8
            self.pNSW = 1. / 3
            self.pSN = 1. / 8
            self.pSNE = 1. / 3
            self.pWEN = 1. / 4
            self.pWES = 1. / 4
            self.pEWS = 1. / 4
            self.pEWN = 1. / 4
        if scenario == "FC_NS_LC_WE":
            self.pWE = 1. / 8 
            self.pEW = 1. / 8
            self.pNS = 1. / 16
            self.pNSW = 1. / 3
            self.pSN = 1. / 16
            self.pSNE = 1. / 3
            self.pWEN = 1. / 4
            self.pWES = 1. / 4
            self.pEWS = 1. / 4
            self.pEWN = 1. / 4            
        if scenario == "FC_ALL":
            self.pWE = 1. / 32 
            self.pEW = 1. / 32
            self.pNS = 1. / 32
            self.pNSW = 1. / 3
            self.pSN = 1. / 32
            self.pSNE = 1. / 3
            self.pWEN = 1. / 4
            self.pWES = 1. / 4
            self.pEWS = 1. / 4
            self.pEWN = 1. / 4                       
        
def generate_routefile(scenario, outputfile):
    random.seed(42)  # make tests reproducible
    N = 3600  # number of time steps
    # demand per second from different directions
    # pWE = 1. / 4 old benchmarks
    # pEW = 1. / 4
    # pNS = 1. / 8
    # pNSW = 1. / 3
    # pSN = 1. / 8
    # pSNE = 1. / 3
    # pWEN = 1. / 4
    # pWES = 1. / 4
    # pEWS = 1. / 4
    # pEWN = 1. / 4
    cf = Configuration(scenario)
    pWE = cf.pWE
    pEW = cf.pEW
    pNS = cf.pNS
    pNSW = cf.pNSW
    pSN = cf.pSN
    pSNE = cf.pSNE
    pWEN = cf.pWEN
    pWES = cf.pWES
    pEWS = cf.pEWS
    pEWN = cf.pEWN
    
    with open("data/"+outputfile, "w") as routes:
        totalCars = 0;
        print("""<routes>
        <vType id="typeWE" accel="0.8" decel="4.5" sigma="0.5" length="5" minGap="2.5" maxSpeed="16.67" guiShape="passenger"/>
	<vType id="typeNS" accel="0.8" decel="4.5" sigma="0.5" length="5" minGap="2.5" maxSpeed="16.67" guiShape="passenger"/>
	<route id="left" edges="366845096#0 366845096#1 366845096#20 366845096#22 366845096#3 366845096#4 366845096#5"/>
	<route id="left_up" edges="366845096#0 366845096#1 366845096#20 366845096#22 -157076458#1 -157076458#0"/>
	<route id="left_down" edges="366845096#0 366845096#1 366845096#20 366845096#22 -26446721#2 -26446721#1 -26446721#0"/>
	<route id="right" edges="-366845096#5 -366845096#4 -366845096#30 -366845096#32 -366845096#2 -366845096#1 -366845096#0 "/>
	<route id="right_up" edges="-366845096#5 -366845096#4 -366845096#30 -366845096#32 -157076458#1 -157076458#0 "/>
	<route id="right_down" edges="-366845096#5 -366845096#4 -366845096#30 -366845096#32 -26446721#2 -26446721#1 -26446721#0"/>
	<route id="up" edges="157076458#0 157076458#1 -26446721#2 -26446721#1 -26446721#0"/>
	<route id="up_left" edges="157076458#0 157076458#1 -366845096#2 -366845096#1 -366845096#0"/>
	<route id="down" edges="26446721#0 26446721#1 26446721#2 -157076458#1 -157076458#0"/>
	<route id="down_right" edges="26446721#0 26446721#1 26446721#2 366845096#3 366845096#4 366845096#5"/>""", file=routes)
        lastVeh = 0
        vehNr = 0
        for i in range(N):
            if random.uniform(0, 1) < pWE:
                if random.uniform(0, 1) < pWES:  # goes down
                    totalCars = totalCars + 1
                    print('    <vehicle id="right_%i" type="typeWE" route="right_down" depart="%i" />' % (
                    vehNr, i), file=routes)
                else:
                    if random.uniform(0, 1) < pWES:  # goes up, this is the leats probably 0.18
                        totalCars = totalCars + 1
                        print('    <vehicle id="right_%i" type="typeWE" route="right_up" depart="%i" />' % (
                            vehNr, i), file=routes)
                    else:
                        totalCars = totalCars + 1
                        print('    <vehicle id="right_%i" type="typeWE" route="right" depart="%i" />' % (
                            vehNr, i), file=routes)
                vehNr += 1
                lastVeh = i
            if random.uniform(0, 1) < pEW:
                if random.uniform(0, 1) < pEWN:  # goes up
                    totalCars = totalCars + 1
                    print('    <vehicle id="left_%i" type="typeWE" route="left_up" depart="%i" />' % (
                        vehNr, i), file=routes)
                else:
                    if random.uniform(0, 1) < pEWS:  # goes down, this is the leats probably 0.18
                        totalCars = totalCars + 1
                        print('    <vehicle id="right_%i" type="typeWE" route="left_down" depart="%i" />' % (
                            vehNr, i), file=routes)
                    else:
                        totalCars = totalCars + 1                        
                        print('    <vehicle id="right_%i" type="typeWE" route="left" depart="%i" />' % (
                            vehNr, i), file=routes)                
                vehNr += 1
                lastVeh = i
            if random.uniform(0, 1) < pSN:
                if random.uniform(0, 1) < pSNE:  # goes right
                    totalCars = totalCars + 1
                    print('    <vehicle id="up_%i" type="typeWE" route="down_right" depart="%i" color="1,0,0"/>' % (
                        vehNr, i), file=routes)
                else:
                    totalCars = totalCars + 1
                    print('    <vehicle id="up_%i" type="typeWE" route="down" depart="%i" color="1,0,0"/>' % (
                        vehNr, i), file=routes)
                vehNr += 1
                lastVeh = i
            if random.uniform(0, 1) < pNS:
                if random.uniform(0, 1) < pNSW:  # goes left
                    totalCars = totalCars + 1
                    print('    <vehicle id="down_%i" type="typeWE" route="up_left" depart="%i" color="1,0,0"/>' % (
                        vehNr, i), file=routes)
                else:
                    totalCars = totalCars + 1
                    print('    <vehicle id="down_%i" type="typeWE" route="up" depart="%i" color="1,0,0"/>' % (
                        vehNr, i), file=routes)
                vehNr += 1
                lastVeh = i
        print('<!-- totalCars="%i -->' % (totalCars), file=routes)        
        print("</routes>", file=routes)

def get_options():
    optParser = optparse.OptionParser()
    optParser.add_option("--scenario", type="string", dest="scenario",default="LC_ALL")
    optParser.add_option("--output", type="string", dest="output",default="nylandsvejPlain.rou.xml")
    options, args = optParser.parse_args()
    return options

# this is the main entry point of this script
if __name__ == "__main__":
    options = get_options()
    generate_routefile(options.scenario,options.output)
