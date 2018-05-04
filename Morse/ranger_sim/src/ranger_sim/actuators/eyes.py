import logging; logger = logging.getLogger("morse." + __name__)

from morse.core.actuator import Actuator
from morse.core import mathutils
from morse.helpers.components import add_data


class Eyes(Actuator):

    _name = "Eyes"
    _short_desc = "Controls the eyes of the EPFL Ranger robot"

    add_data('left', 0.1, 'float', 'Left eye rotation, in radians')
    add_data('right', -0.1, 'float', 'Right eye rotation, in radians')

    def __init__(self, obj, parent=None):
        logger.info("%s initialization" % obj.name)
        # Call the constructor of the parent class

	
	Actuator.__init__(self, obj, parent=)

	self.left.eye = parent.bge_object,children["left_eye"]
	self.right.eye = parent.bge_object,children["right_eye"]

        logger.info('Component initialized')

    def default_action(self):
	
	l_orientation = mathutils.Euler([self.local_data['left'],0.0,0.0)
	self.left_eye.orientation = l_orientation_to_matrix()

	r_orientation = mathutils.Euler([self.local_data['right'],0.0,0.0)
	self.right_eye.orientation = r_orientation_to_matrix()
