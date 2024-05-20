#! /usr/bin python

import sys, getopt
import rospy
from std_msgs.msg import String, Bool
import json
from tulip import synth
import gr1strategy.aux as aux
from gr1strategy.srv import GetSensors, SetActuators
import time

class StrategyExecutor():
    """ 
    Executor for FSM controller strategy
    This class provides functions to update the outputs and to check for new states in every iteration.
    """
    def __init__(self, strat):
        self.strategy = strat

        self.current_state = next(iter(self.strategy.states.initial))
        self.inputs = self.strategy.inputs
        self.outputs = self.strategy.outputs

        # self.actuator_pub = rospy.Publisher("/actuators", String, queue_size=10) 
        # self.sensor_pub = rospy.Publisher("/sensors_props", String, queue_size=10)

        self.states_seq = [self.current_state]
        self.output_seq = []

        # self.runStrategy = threading.Event()  # Start out paused
        # self.alive = threading.Event()
        # self.alive.set()

    def setActuators(self, outputs:dict) -> bool:
        """
        Execute actuators based on output proposition values
        """

        rospy.wait_for_service('set_actuators')
        try:
            set_actuators = rospy.ServiceProxy('set_actuators', SetActuators)
            resp = set_actuators(json.dumps(outputs))
            return resp.feedback
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s"%e)

    def getSensorValues(self) -> dict:
        
        rospy.wait_for_service('get_sensors')
        try:
            get_sensors = rospy.ServiceProxy('get_sensors', GetSensors)
            resp = get_sensors(self.inputs)
            input_props = json.loads(str(resp.sensors_values))
            return input_props
            
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s"%e)

    def runStrategyIteration(self):
        # run the FSM for one evaluation step
        rospy.loginfo("=============== Strategy Iteration ===============") 
        
        ######## Sensor Value ########
        input_props = self.getSensorValues()
    
        ######## Transition ########
        # Find next states
        # next_states = self.strategy.findTransitionableStates(input_props, from_state=self.strategy.current_state)
        # get current state from sensor data
        next_state, output_props = self.strategy.reaction(self.current_state, input_props)

        ######## Set Actuation and receive feedback ########
        act_feedback = self.setActuators(output_props)

        ######## Update Strategy ########
        if act_feedback:
            rospy.loginfo("Transition complete: form state {} to state {}".format(str(self.current_state), str(next_state)))
            self.current_state = next_state

            self.states_seq.append(self.current_state)
            self.output_seq.append(output_props)
        else:
            rospy.loginfo("Transition not completed: state remains in {}".format(str(self.current_state)))

    def run(self):

        while 1:
            self.runStrategyIteration()
            time.sleep(1)

        rospy.logdebug("strategy executor quitting...")


def execute_main(spec_file=None):
    rospy.loginfo("Hello. Let's do this!")

    # Synthesize strategy from spec
    # specs = load_GRSpec("/home/dongyanqi/catkin_ws/src/gr1strategy/specs/demo_gr1.spc")

    specs = None
    if spec_file is not None:
        specs = aux.load_GRSpec(spec_file)
        specs.qinit = r'\E \A'
    else:
        rospy.logerr("Please specify a spec file for synthesis!")
    
    # Mealy
    ctrl = synth.synthesize(specs)
    rospy.loginfo("Control Strategy: {}".format(ctrl))

    e = StrategyExecutor(ctrl)

    # Start the executor's main loop in this thread
    e.run()

    # # Clean up on exit
    # logging.info("Waiting for XML-RPC server to shut down...")
    # xmlrpc_server.shutdown()
    # XMLRPCServerThread.join()
    # logging.info("XML-RPC server shutdown complete.  Goodbye.")


if __name__ == "__main__":
    ### Check command-line arguments

    spec_file = None

    try:
        opts, args = getopt.getopt(sys.argv[1:], "hs:", ["help", "spec-file="])
    except getopt.GetoptError:
        usage(sys.argv[0])
        sys.exit(2)

    for opt, arg in opts:
        if opt in ("-h", "--help"):
            sys.exit()
        elif opt in ("-s", "--spec-file"):
            spec_file = arg

    rospy.init_node("strategy_executor")
    execute_main(spec_file)
    rospy.spin()

