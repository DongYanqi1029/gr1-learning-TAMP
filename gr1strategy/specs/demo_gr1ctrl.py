class Demo_gr1Ctrl:
    """Mealy transducer.

    Internal states are integers, the current state
    is stored in the attribute "state".
    To take a transition, call method "move".

    The names of input variables are stored in the
    attribute "input_vars".

    Automatically generated by tulip.dumpsmach on 2024-04-23 02:47:30 UTC
    To learn more about TuLiP, visit http://tulip-control.org
    """
    def __init__(self):
        self.state = 5
        self.input_vars = ['']

    def move(self, ) -> dict:
        """Given inputs, take move and return outputs.

        @return:
            dictionary with keys of the output variable names:
            ['r3', 'r2', 'r1']
        """
        output = dict()
        if self.state == 0:
            if True:
                self.state = 1

                output["r3"] = False
                output["r2"] = False
                output["r1"] = True
        elif self.state == 1:
            if True:
                self.state = 2

                output["r3"] = False
                output["r2"] = True
                output["r1"] = False
        elif self.state == 2:
            if True:
                self.state = 3

                output["r3"] = False
                output["r2"] = False
                output["r1"] = True
        elif self.state == 3:
            if True:
                self.state = 4

                output["r3"] = True
                output["r2"] = False
                output["r1"] = False
        elif self.state == 4:
            if True:
                self.state = 0

                output["r3"] = False
                output["r2"] = False
                output["r1"] = True
        elif self.state == 5:
            if True:
                self.state = 0

                output["r3"] = False
                output["r2"] = False
                output["r1"] = True
        else:
            raise AssertionError("Unrecognized internal state: " + str(self.state))
        return output

    def _error(self, ):
        raise ValueError("Unrecognized input: " + ().format())
