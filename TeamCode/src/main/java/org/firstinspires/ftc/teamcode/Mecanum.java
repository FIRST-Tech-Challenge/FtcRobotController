package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Basic Mecanum Drive")
public class Mecanum extends LinearOpMode {

  private DcMotor frontLeftMotor;
  private DcMotor backRightMotor;
  private DcMotor backLeftMotor;
  private DcMotor frontRightMotor;
  
  // private DcMotor armMotor;
  
  /**
   * This function is executed when this Op Mode is selected from the Driver Station.
   */
  @Override
  public void runOpMode() {
    frontLeftMotor = hardwareMap.get(DcMotor.class, "frontLeftMotor");
    backRightMotor = hardwareMap.get(DcMotor.class, "backRightMotor");
    backLeftMotor = hardwareMap.get(DcMotor.class, "backLeftMotor");
    frontRightMotor = hardwareMap.get(DcMotor.class, "frontRightMotor");
    
    // armMotor = hardwareMap.get(DcMotor.class, "armMotor");

    // Put initialization blocks here.
    waitForStart();
    if (opModeIsActive()) {
      // Put run blocks here.
      backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
      frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

      while (opModeIsActive()) {
        backLeftMotor.setPower(((0 - gamepad1.right_stick_y) + gamepad1.left_stick_x) - gamepad1.right_stick_x);
        backRightMotor.setPower(((0 - gamepad1.right_stick_y) - gamepad1.left_stick_x) + gamepad1.right_stick_x);
        frontLeftMotor.setPower((0 - gamepad1.right_stick_y) + gamepad1.left_stick_x + gamepad1.right_stick_x);
        frontRightMotor.setPower(((0 - gamepad1.right_stick_y) - gamepad1.left_stick_x) - gamepad1.right_stick_x);


        // if (gamepad1.dpad_up) {
        //   armMotor.set(.25);
        // } else if (gamepad1.dpad_down) {
        //   armMotor.set(-.25); 
        // }
      }
    }
  }
}