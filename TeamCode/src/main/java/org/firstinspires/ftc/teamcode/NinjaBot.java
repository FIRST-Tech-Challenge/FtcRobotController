package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.*;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


public class NinjaBot {
  // Define motors
  public DcMotor leftDrive = null;
  public DcMotor rightDrive = null;

  // define constants and other variabes

  // local OpMode members
  HardwareMap hwMap = null;
  LinearOpMode control = null;
  private ElapsedTime period = new ElapsedTime();

  public NinjaBot(HardwareMap map, LinearOpMode ctrl) {
    init(map, ctrl);
  }

  public void init(HardwareMap map, LinearOpMode ctrl) {
    // Save reference to Hardware map
    hwMap = map;
    control = ctrl;

    // link motors to their references
    leftDrive = hwMap.get(DcMotor.class, "left");
    rightDrive = hwMap.get(DcMotor.class, "right");

    leftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
  }

  // define functions relating to the control of motors of the robot in general.
  // specific control should be in the OpMode code file/s.
  // eg: public void updateWheelTelemetry() {...}
}
