#-------------------------------------------------------------------------------
# Name:        module1
# Purpose:
#
# Author:      Jon
#
# Created:     16/07/2014
# Copyright:   (c) Jon 2014
# Licence:     <your licence>
#-------------------------------------------------------------------------------

def stripExtra(x):
    # Example: {"data":"3-1-182-184-2-3-1-182-167-2-83-1-182-171-2-55-5-255-255-5","ttl":"60","published_at":"2014-07-16T02:24:12.000Z","coreid":"48ff70065067555031451287"}
    x = x.strip("{\"data\":\"")
    x = x[:-(len(x)-x.find("\",\"ttl"))] # strip off part "","ttl" and onward
    return x

def getIntList(x):
    x = stripExtra(x)
    dataList = x.split('-')
    is_integer = lambda s: s.isdigit()
    integers = filter(is_integer, dataList)
    return integers

def getIntRunData(x):
    x = stripExtra(x)
    dataList = x.split()
    is_integer = lambda s: s.isdigit()
    integers = filter(is_integer, dataList)
    return integers

def getBeaconDistance(reading):
    if len(reading) != 5:
        print "Warning: got reading length of", len(reading), "expected 5"
        return
    txpower = int(reading[2])
    rssi = int(reading[3])
    distance = 0.0
    ratio = (256 - rssi) * 1.0 / (256 - txpower)
    if(ratio < 1.0):
        distance = pow(ratio, 10)
    else:
       distance = (0.89976)*pow(ratio,7.7095) + 0.111
    return distance

def printRun(runData, num, readings):
    print "Run:", num, "NumReadings:", readings
    for x in runData:
        print x[0], getBeaconDistance(x)


f = open('raw_map_data.txt')
lines = f.readlines()
f.close()

runNum = -1
numReadings = -1
dataPoints = -1

runData = []

for x in lines:
    if len(x) > 1:
        if "Run: " in x:
            if dataPoints > 0:
                print "RUN NOT COMPLETE!", dataPoints, "missing datapoints"
                printRun(runData, runNum, numReadings)
            runData = []
            runInfo = getIntRunData(x)
            if len(runInfo) != 2:
                print "Warning: got", len(runInfo), "expected 2"
            else:
                runNum = runInfo[0]
                numReadings = runInfo[1]
                dataPoints = int(runInfo[1]) * 5
        else:
            bleMeasurements = getIntList(x)
            if len(bleMeasurements) != 20:
                print "Warning: got", len(bleMeasurements), "expected 20"
            else:
                for x in range(0, 20, 5):
                    runData.append(bleMeasurements[x:x+5])
                    dataPoints -= 5
                    if dataPoints == 0:
                        #print "RUN COMPLETE"
                        printRun(runData, runNum, numReadings)
                        break


