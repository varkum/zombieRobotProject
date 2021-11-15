from controller import Robot, Motor

LEFT = 0
RIGHT= 1

MIN_POS = 0.0
MAX_POS = 0.025
OFFSET_WHEN_LOCKED = 0.021

class Gripper(Robot):
  def __init__(self):
        super(Gripper, self).__init__()
        self.fingers[LEFT] = self.getDevice('finger1')
        self.fingers[RIGHT] = self.getDevice('finger2')

        self.fingers[LEFT].setVelocity(0.03)
        self.fingers[RIGHT].setVelocity(0.03)

  def gripper_grip(self):
    self.fingers[LEFT].setPosition(float(MIN_POS))
    self.fingers[RIGHT].setPosition(float(MIN_POS))


def gripper_release(self):
  self.fingers[LEFT].setPosition(float(MAX_POS))
  self.fingers[RIGHT].setPosition(float(MAX_POS))


def gripper_set_gap(self, double gap):
  #implement bound function v = bound(0.5 * (gap - OFFSET_WHEN_LOCKED), MIN_POS, MAX_POS);
  self.fingers[LEFT].setPosition(float(v))
  self.fingers[RIGHT].setPosition(float(v))

