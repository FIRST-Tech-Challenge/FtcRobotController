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

  //Move linear Slides Up (Value has to be postive 0.0-1.0)
  public void up() {

    //Move linear slide all the way up with max power
    up(1, 1);

  }

  public void up(double power) {

    //Moves linear slide all the way up with set power.
    up(power,1);

  }

  public void up(double power, double distance) {

    //Moves linear slide up set distance and speed.
    Drive.setPower(power);


  }
  
  
  //Moves linear slide down (Value has to be postive 0.0-1.0)
  public void down() {

    //Move linear slide all the way down with max power
    Drive.setPower(-1);

  }

  public void down(double power) {

    //Moves linear slide all the way down with set power.
    Drive.setPower(-power);

  }

  public void down(double power, double distance) {

    //Moves linear slide down set distance and speed.
    Drive.setPower(-power);

  }

}
