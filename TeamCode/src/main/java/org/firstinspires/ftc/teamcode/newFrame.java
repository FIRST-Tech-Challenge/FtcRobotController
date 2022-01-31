// Created by Vikram Kommera on 10/8/2021
// Mechanum Wheel Control
// Inspired by https://gm0.org/en/latest/docs/software/mecanum-drive.html

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "newFrame", group = "16481")
public class newFrame extends LinearOpMode {
   DcMotor motorFrontLeft;
   DcMotor motorBackLeft;
   DcMotor motorFrontRight;
   DcMotor motorBackRight;

   double y, x, rx ;
   double denominator;
   double frontLeftPower, backLeftPower, frontRightPower, backRightPower;

   @Override
   public void runOpMode() throws InterruptedException {
      motorFrontLeft = hardwareMap.dcMotor.get("motorFrontLeft");
      motorBackLeft = hardwareMap.dcMotor.get("motorBackLeft");
      motorFrontRight = hardwareMap.dcMotor.get("motorFrontRight");
      motorBackRight = hardwareMap.dcMotor.get("motorBackRight");

      motorFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);

      waitForStart();

      if (isStopRequested()) return;

      while (opModeIsActive()) {
         y = -gamepad1.left_stick_y; // Remember, this is reversed!
         x = gamepad1.left_stick_x * 0.98; // Counteract imperfect strafing
         rx = gamepad1.right_stick_x/2*-1; //increase speed by 2

         denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
         frontLeftPower = (y + x + rx) / denominator;
         backLeftPower = (y - x + rx) / denominator;
         frontRightPower = (y - x - rx) / denominator;
         backRightPower = (y + x - rx) / denominator;

         motorFrontLeft.setPower(frontLeftPower);
         motorBackLeft.setPower(backLeftPower);
         motorFrontRight.setPower(frontRightPower);
         motorBackRight.setPower(backRightPower);

         telemetry.addData("# front left", motorFrontLeft.getPower());
         telemetry.addData("# front right", motorFrontRight.getPower());
         telemetry.addData("# back left", motorBackLeft.getPower());
         telemetry.addData("# back right", motorBackRight.getPower());

         telemetry.update();

      }
   }
}