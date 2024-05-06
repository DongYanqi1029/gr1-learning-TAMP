import aux
import sys, getopt
import os 
from tulip import synth

if __name__ == "__main__":
    opts, args = getopt.getopt(sys.argv[1:], '-f:', ["file="])

    file = ''
    for opt,arg in opts:
        if opt in ('-f', '--file'):
            file = arg

    # Synthesize strategy from spec file
    filepath = os.path.dirname(file)
    filename = os.path.splitext(os.path.basename(file))[0]

    specs = aux.load_GRSpec(file)
    specs.qinit = r'\E \A'

    ctrl = synth.synthesize(specs)

    # # Dump strategy as a xml file
    # aux.dump_strategy_class(filepath, filename, ctrl)
