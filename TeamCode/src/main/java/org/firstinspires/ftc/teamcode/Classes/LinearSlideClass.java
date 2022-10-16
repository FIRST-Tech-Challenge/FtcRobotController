package org.firstinspires.ftc.teamcode.Classes;
//brendan
public class LinearSlideClass {
  leftDrive  = hardwareMap.get(DcMotor.class, "left_drive");
  rightDrive = hardwareMap.get(DcMotor.class, "right_drive");

  leftDrive.setDirection(DcMotor.Direction.FORWARD);
  rightDrive.setDirection(DcMotor.Direction.REVERSE);
}
