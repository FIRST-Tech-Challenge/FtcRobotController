package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;


public class NinjaBot {

  // Define motors
  // eg: public DcMotor leftDrive = null;

  // define constants and other variabes

  // local OpMode members
  HardwareMap hwMap =  null;
  LinearOpMode control =  null;
  private ElapsedTime period  = new ElapsedTime();

  public Ninjabot(HardwareMap map, LinearOpMode ctrl ){
    init(map, ctrl);
  }

  public void init(HardwareMap map, LinearOpMode ctrl) {
    // Save reference to Hardware map
    hwMap = map;
    control = ctrl;

    // link motors to their references
    // eg: leftDrive = hwMap.get(DcMotor.class, "RD");
  }
  // define functions relating to the controll of motors of the robot in general. specific controll should be in the OpMode code file/s.
  // eg: public void updateWheelTelemetry() {...}
}