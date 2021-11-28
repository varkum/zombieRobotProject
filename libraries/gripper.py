from controller import Robot, Motor
from tiny_math import bound

LEFT = 0
RIGHT= 1

MIN_POS = 0.0
MAX_POS = 0.025
OFFSET_WHEN_LOCKED = 0.021

class Gripper(Robot):
  def __init__(self):
        super(Gripper, self).__init__()
        #fingers[0] is LEft, 1 is rIGHT
        self.fingers = []
        self.fingers.append(self.getDevice('finger1'))
        self.fingers.append(self.getDevice('finger2'))

        self.fingers[LEFT].setVelocity(0.03)
        self.fingers[RIGHT].setVelocity(0.03)

  def grip(self):
    self.fingers[LEFT].setPosition(float(MIN_POS))
    self.fingers[RIGHT].setPosition(float(MIN_POS))


  def release(self):
   self.fingers[LEFT].setPosition(float(MAX_POS))
    self.fingers[RIGHT].setPosition(float(MAX_POS))


  def setGap(self, double gap):
    v = bound(0.5 * (gap - OFFSET_WHEN_LOCKED), MIN_POS, MAX_POS);
    self.fingers[LEFT].setPosition(float(v))
    self.fingers[RIGHT].setPosition(float(v))

