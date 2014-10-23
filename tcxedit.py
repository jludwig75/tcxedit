import sys
import os

from tcxparser import TcxFile
from woprocessor import WorkOutProcessor
from options import *
            
def main(programName, args):
    try:
        options = ProgramOptions.ParseCommandLine(sys.argv[1:])
    except Exception as e:
        print '%s\n' % e
        ProgramOptions.Help()
        return -1
    
    stopTime = 0
    #if len(args) > 1:
    #    stopTime = int(args[1])
    
    tcxFile = TcxFile.ParseFile(options.FileName(), stopTime)
    
    wop = WorkOutProcessor(tcxFile.activities, options)

    wop.DumpActivitiesByLaps()
    #wop.DumpActivities()
    
    if stopTime != 0:
        tcxFile.write('new.tcx')
    
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv[0], sys.argv[1:])) 