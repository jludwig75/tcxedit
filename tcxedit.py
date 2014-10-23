import sys
import os

from tcxparser import *
from woprocessor import *

            
def main(programName, args):
    TestForceCaclulations()
    fileName = args[0]
    
    stopTime = 0
    if len(args) > 1:
        stopTime = int(args[1])
    
    print 'Processing file "%s"' % fileName
    
    tcxFile = TcxFile.ParseFile(fileName, stopTime)
    
    wop = WorkOutProcessor(tcxFile.activities)

    wop.DumpActivitiesByLaps()
    #wop.DumpActivities()
    
    if stopTime != 0:
        tcxFile.write('new.tcx')
    
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv[0], sys.argv[1:])) 