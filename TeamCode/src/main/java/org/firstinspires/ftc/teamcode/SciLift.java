package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode. TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

public class SciLift extends LinearOpMode {

  //the lift is controlled by one motor.
  public DcMotor motor = null;

  public SciLift(DcMotor mtr) {
    motor = mtr;
    motor.setDirection(DcMotor.Direction.REVERSE);
    //resetting the encoder. used in autonomous to control the lift motor.
    motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    //motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
  }

  //there are two "up" and two "down" functions.
  //one takes parameters controlling speed, the other one is default
  //the parameters should be between 0 and 1

  public void down(float power) {
    motor.setPower(0.6 * power);
  }

  public void down() {
    motor.setPower(0.6);
  }
  
  public void up(float power) {
    motor.setPower(-1 * power);
  }

  public void up() {
    motor.setPower(-1);
  }

  public void rest() {
    motor.setPower(0);
  }

  public double getPower() {
    double p = motor.getPower();
    return p;
  }

  public double getClicks() {
    double c = motor.getCurrentPosition();
    return c;
  }

  public void runOpMode() {
  }
}
