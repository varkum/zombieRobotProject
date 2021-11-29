from controller import Robot, Motor

ARM1, ARM2, ARM3, ARM4, ARM5 = 0, 1, 2, 3, 4



class Arm(Robot):
  def __init(self)__:
    super(Arm, self).__init__()
    self.armElements = []
    self.armElements.append(self.getDevice("arm1"))
    self.armElements.append(self.getDevice("arm2"))
    self.armElements.append(self.getDevice("arm3"))
    self.armElements.append(self.getDevice("arm4"))
    self.armElements.append(self.getDevice("arm5"))


  self.armElements[ARM2].setVelocity(0.5)

  armSetHeight("ARM_RESET")
  armSetOrientation("ARM_FRONT")


  def armReset(self):
    self.armElements[ARM1].setPosition(0.0)
    self.armElements[ARM2].setPosition(1.57)
    self.armElements[ARM3].setPosition(-2.635)
    self.armElements[ARM4].setPosition(1.78)
    self.armElements[ARM5].setPosition(0.0)


  def armSetHeight(self, height):
    
    case ARM_FRONT_FLOOR:
      wb_motor_set_position(arm_elements[ARM2], -0.97);
      wb_motor_set_position(arm_elements[ARM3], -1.55);
      wb_motor_set_position(arm_elements[ARM4], -0.61);
      wb_motor_set_position(arm_elements[ARM5], 0.0);
      break;
    case ARM_FRONT_PLATE:
      wb_motor_set_position(arm_elements[ARM2], -0.62);
      wb_motor_set_position(arm_elements[ARM3], -0.98);
      wb_motor_set_position(arm_elements[ARM4], -1.53);
      wb_motor_set_position(arm_elements[ARM5], 0.0);
      break;
    case ARM_FRONT_CARDBOARD_BOX:
      wb_motor_set_position(arm_elements[ARM2], 0.0);
      wb_motor_set_position(arm_elements[ARM3], -0.77);
      wb_motor_set_position(arm_elements[ARM4], -1.21);
      wb_motor_set_position(arm_elements[ARM5], 0.0);
      break;
    case ARM_RESET:
      wb_motor_set_position(arm_elements[ARM2], 1.57);
      wb_motor_set_position(arm_elements[ARM3], -2.635);
      wb_motor_set_position(arm_elements[ARM4], 1.78);
      wb_motor_set_position(arm_elements[ARM5], 0.0);
      break;
    case ARM_BACK_PLATE_HIGH:
      wb_motor_set_position(arm_elements[ARM2], 0.678);
      wb_motor_set_position(arm_elements[ARM3], 0.682);
      wb_motor_set_position(arm_elements[ARM4], 1.74);
      wb_motor_set_position(arm_elements[ARM5], 0.0);
      break;
    case ARM_BACK_PLATE_LOW:
      wb_motor_set_position(arm_elements[ARM2], 0.92);
      wb_motor_set_position(arm_elements[ARM3], 0.42);
      wb_motor_set_position(arm_elements[ARM4], 1.78);
      wb_motor_set_position(arm_elements[ARM5], 0.0);
      break;
    case ARM_HANOI_PREPARE:
      wb_motor_set_position(arm_elements[ARM2], -0.4);
      wb_motor_set_position(arm_elements[ARM3], -1.2);
      wb_motor_set_position(arm_elements[ARM4], -M_PI_2);
      wb_motor_set_position(arm_elements[ARM5], M_PI_2);
      break;
    default:
      fprintf(stderr, "arm_height() called with a wrong argument\n");
      return;
  }
  current_height = height;
}



void arm_increase_height() {
  current_height++;
  if (current_height >= ARM_MAX_HEIGHT)
    current_height = ARM_MAX_HEIGHT - 1;
  arm_set_height(current_height);
}

void arm_decrease_height() {
  current_height--;
  if ((int)current_height < 0)
    current_height = 0;
  arm_set_height(current_height);
}



void arm_decrease_orientation() {
  current_orientation--;
  if ((int)current_orientation < 0)
    current_orientation = 0;
  arm_set_orientation(current_orientation);
}

void arm_set_sub_arm_rotation(enum Arm arm, double radian) {
  wb_motor_set_position(arm_elements[arm], radian);
}

double arm_get_sub_arm_length(enum Arm arm) {
  switch (arm) {
    case ARM1:
      return 0.253;
    case ARM2:
      return 0.155;
    case ARM3:
      return 0.135;
    case ARM4:
      return 0.081;
    case ARM5:
      return 0.105;
  }
  return 0.0;
}

void arm_ik(double x, double y, double z) {
  double x1 = sqrt(x * x + z * z);
  double y1 = y + arm_get_sub_arm_length(ARM4) + arm_get_sub_arm_length(ARM5) - arm_get_sub_arm_length(ARM1);

  double a = arm_get_sub_arm_length(ARM2);
  double b = arm_get_sub_arm_length(ARM3);
  double c = sqrt(x1 * x1 + y1 * y1);

  double alpha = -asin(z / x1);
  double beta = -(M_PI_2 - acos((a * a + c * c - b * b) / (2.0 * a * c)) - atan(y1 / x1));
  double gamma = -(M_PI - acos((a * a + b * b - c * c) / (2.0 * a * b)));
  double delta = -(M_PI + (beta + gamma));
  double epsilon = M_PI_2 + alpha;

  wb_motor_set_position(arm_elements[ARM1], alpha);
  wb_motor_set_position(arm_elements[ARM2], beta);
  wb_motor_set_position(arm_elements[ARM3], gamma);
  wb_motor_set_position(arm_elements[ARM4], delta);
  wb_motor_set_position(arm_elements[ARM5], epsilon);
}
