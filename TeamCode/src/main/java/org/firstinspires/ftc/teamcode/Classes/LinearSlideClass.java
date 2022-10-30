package org.firstinspires.ftc.teamcode.Classes;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

// initializes a
public class LinearSlideClass {

  public DcMotorEx Drive;
  
  //Initializes the Linear Slide
  public void initializeLinearSlide() {

    Drive  = hardwareMap.get(DcMotorEx.class, "drive");

    Drive.setDirection(DcMotor.Direction.FORWARD);
  }

  //Move linear Slides Up for a certain amount of milliseconds (Value has to be positive 0.0-1.0)
  public void linearSlideUp(double power, int time) throws InterruptedException {

    //Moves linear slide all the way up with set power.
    Drive.setPower(power);

    Thread.sleep(time);

    Drive.setPower(0);

  }




  //Move linear Slides Up for a certain amount of milliseconds (Value has to be negative -1.0-0.0)
  public void linearSlideDown(double power, int time) throws InterruptedException {

    //Moves linear slide all the way up with set power.
    Drive.setPower(power);

    Thread.sleep(time);

    Drive.setPower(0);

  }

}
