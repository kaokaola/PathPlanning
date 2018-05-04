from morse.builder.creator import ActuatorCreator

class Eyes(ActuatorCreator):
    _classpath = "ranger_sim.actuators.eyes.Eyes"
    _blendname = "eyes"

    def __init__(self, name=None):
        ActuatorCreator.__init__(self, name)

