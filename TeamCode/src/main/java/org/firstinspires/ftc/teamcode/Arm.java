package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

public class Arm extends LinearOpMode {

  private DcMotor motor = null;
  public String state = "rest";


  public Arm(DcMotor mtr) {
    motor = mtr;
    motor.setDirection(DcMotor.Direction.REVERSE);
  }

  public void retract(double power) {
    motor.setPower(1 * power);
  }

  public void retract() {
    motor.setPower(1);
  }

  public void extend(double power) {
    motor.setPower(-1 * power);
  }

  public void extend() {
    motor.setPower(-1);
  }

  public void rest() {
    motor.setPower(0);
  }

  public double getPower() {
    double p = motor.getPower();
    return p;
  }

  public void runOpMode() {
  }
}
