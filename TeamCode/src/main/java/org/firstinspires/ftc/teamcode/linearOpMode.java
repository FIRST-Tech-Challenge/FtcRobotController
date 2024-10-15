package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Mecanum Drive")
public class linearOpMode extends LinearOpMode {
  private DcMotor frontLeftMotor = null, backLeftMotor = null;
  private DcMotor frontRightMotor = null, backRightMotor = null;


  @Override
  public void runOpMode(){


    frontLeftMotor = hardwareMap.get(DcMotor.class, "leftFront");
    frontRightMotor = hardwareMap.get(DcMotor.class, "rightFront");
    backLeftMotor = hardwareMap.get(DcMotor.class, "leftBack");
    backRightMotor = hardwareMap.get(DcMotor.class, "rightBack");

    frontLeftMotor.setDirection(DcMotor.Direction.FORWARD);
    frontRightMotor.setDirection(DcMotor.Direction.REVERSE);
    backLeftMotor.setDirection(DcMotor.Direction.FORWARD);
    backRightMotor.setDirection(DcMotor.Direction.REVERSE);


    waitForStart();

    if (isStopRequested()) return;

    while (opModeIsActive()) {

      double y = -gamepad1.left_stick_y;
      double x = -gamepad1.left_stick_x ;
      double turn = gamepad1.right_stick_x;

      //input: theta and power
      //theta is where we want the direction the robot to go
      //power is (-1) to 1 scale where increasing power will cause the engines to go faster
      double theta = Math.atan2(y,x);
      double power = Math.hypot(x,y);
      double sin = Math.sin(theta - Math.PI/4);
      double cos = Math.cos(theta - Math.PI/4);
      //max variable allows to use the motors at it's max power with out disabling it
      double max = Math.max(Math.abs(sin),Math.abs(cos));

      double leftFront = power * cos/max + turn;
      double rightFront = power * cos/max - turn;
      double leftRear = power * sin/max + turn;
      double rightRear = power * sin/max - turn;


      //Prevents the motors exceeding max power thus motors will not seize and act sporadically
      if ((power + Math.abs(turn))>1){
        leftFront /= power  + turn;
        rightFront /= power  - turn;
        leftRear /= power  + turn;
        rightRear /= power  - turn;
      }


      frontLeftMotor.setPower(leftFront);
      backLeftMotor.setPower(leftRear);
      frontRightMotor.setPower(rightFront);
      backRightMotor.setPower(rightRear);
    }
  }
}


