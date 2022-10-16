package org.firstinspires.ftc.teamcode.Classes;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

//brendan
public class LinearSlideClass {

  public DcMotorEx leftDrive;
  public DcMotorEx rightDrive;

  public void initializeLinearSlide() {

    leftDrive  = hardwareMap.get(DcMotorEx.class, "left_drive");
    rightDrive = hardwareMap.get(DcMotorEx.class, "right_drive");

    leftDrive.setDirection(DcMotor.Direction.FORWARD);
    rightDrive.setDirection(DcMotor.Direction.REVERSE);
  }

  public void linearSlideUp() {



  }

  public void linearSlideDown() {




  }

}
