package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


public class Drive extends LinearOpMode {

  //initializing motors and variables
  public double powerlf;
  public DcMotor motorlf = null;
  public double powerlb;
  public DcMotor motorlb = null;
  public double powerrf;
  public DcMotor motorrf = null;
  public double powerrb;
  public DcMotor motorrb = null;

  //initialized in order to "get" them in other parts of tbe code.
  public double LyInput;
  public double LxInput;
  public double RxInput;
  public double TurboInput;

  public Drive(DcMotor lf, DcMotor lb, DcMotor rf, DcMotor rb) {
    //the setdirections will change based on how the wheels are set up.
    powerlf = 0;
    motorlf = lf;
    motorlf.setDirection(DcMotor.Direction.FORWARD);
    powerlb = 0;
    motorlb = lb;
    motorlb.setDirection(DcMotor.Direction.FORWARD);
    powerrf = 0;
    motorrf = rf;
    motorrf.setDirection(DcMotor.Direction.REVERSE);
    powerrb = 0;
    motorrb = rb;
    motorrb.setDirection(DcMotor.Direction.REVERSE);

    motorlf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    motorlf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
  }

  public void resetEncoderlf() {
    motorlf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    motorlf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
  }

  public void setPower(double Ly, double Lx, double Rx, double Trigger) {
    //using the three controller inputs, we calculate the powers
    //that the motors need in order to move.
    //Ly = forward and backward
    //Lx = left and right
    //Rx = clockwise and counterclockwise
    LyInput = Ly;
    LxInput = Lx;
    RxInput = Rx;
    TurboInput = Trigger;
    double leftfront = (Ly - Lx + Rx)/3;
    double leftback = (Ly + Lx + Rx)/3;
    double rightfront = (Ly - Lx - Rx)/3;
    double rightback = (Ly + Lx - Rx)/3;
    //because we divided the powers by 3, they're somewhere between 0.333 and 1.
    //if we want the motors to always be running at top speed, and still move in
    //the right directions, we divide each number by the highest number
    //as seen in the math below.
    //the other piece is the trigger value. this is from 0 to 1, based on
    //controller input. this value scales all of the powers, meaning that
    //the robot will move between 1% and 100% speed based on how hard the
    //trigger is held down. think of it like a gas pedal in a car.
    if (Trigger > 0) {
      double max = findMax(leftfront,leftback,rightfront,rightback);
      max = max / Trigger;
      max = Math.abs(max);
      leftfront = leftfront / max;
      leftback = leftback / max;
      rightfront = rightfront / max;
      rightback = rightback / max;
    }
    //here we just set the power.
    powerlf = leftfront;
    motorlf.setPower(leftfront);
    powerlb = leftback;
    motorlb.setPower(leftback);
    powerrf = rightfront;
    motorrf.setPower(rightfront);
    powerrb = rightback;
    motorrb.setPower(rightback);
  }

  //the following are get commands that return the values of the class' variables.

  public double getPowerlf() {
    double lf = motorlf.getPower();

    return powerlf;
  }

  public double getPowerlb() {
    double lb = motorlb.getPower();

    return powerlb;
  }

  public double getPowerrf() {
    double rf = motorrf.getPower();

    return powerrf;
  }

  public double getPowerrb() {
    double rb = motorrb.getPower();

    return powerrb;
  }
  
    //We were only using one encoder to read the robot's position in autonomous. Changed to include all 4 motors for testing.
  public double getClickslf() {
    return motorlf.getCurrentPosition();
  }
 
  public double getClickslb() {
    return motorlb.getCurrentPosition();
  }

  public double getClicksrf() {
    return motorrf.getCurrentPosition();
  }
  
  public double getClicksrb() {
    return motorrb.getCurrentPosition();
  }
  

  //this function just compares all of the powers and returns the largest value.
  public double findMax(double lf,double lb,double rf,double rb) {
    if (lf >= lb && lf >= rf && lf >= rb) {
      return lf;
    }
    else if (lb >= lf && lb >= rf && lb >= rb) {
      return lb;
    }
    else if (rf >= lf && rf >= lb && rf >= rb) {
      return rf;
    }
    else if (rb >= lf && rb >= lb && rb >= rf) {
      return rb;
    }
    else {
      return 1;
    }
  }

  public double getLy() {
    return LyInput;
  }
  public double getLx() {
    return LxInput;
  }
  public double getRx() {
    return RxInput;
  }
  public double getTurbo() {
    return TurboInput;
  }

  //runopmode is always needed at the bottom of our classes because reasons.
  public void runOpMode() {
  }
}
