package org.firstinspires.ftc.robotcontroller.external.samples.oursamples202223;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;



@TeleOp(name = "Bonga")

public class Bonga extends LinearOpMode {
  private DcMotor Motor1;
  private DcMotor Motor2;
  private DcMotor Motor3;
  private DcMotor Motor4;
  

  public void runOpMode() {
    Motor1 = hardwareMap.get(DcMotor.class, "Motor1");
    Motor2 = hardwareMap.get(DcMotor.class, "Motor2");
    Motor3 = hardwareMap.get(DcMotor.class, "Motor3");
    Motor4 = hardwareMap.get(DcMotor.class, "Motor4");

    // Put initialization blocks here.
    waitForStart();
    if (opModeIsActive()) {
        while (opModeIsActive()) {
        if (Math.abs(gamepad1.right_stick_y) > Math.abs(gamepad1.right_stick_x)) {
          telemetry.addData("It is working", 123);
          Motor1.setPower(gamepad1.right_stick_y * -1);
          Motor2.setPower(gamepad1.right_stick_y * 1);
          Motor3.setPower(gamepad1.right_stick_y * -1);
          Motor4.setPower(gamepad1.right_stick_y * 1);
        } else if (Math.abs(gamepad1.right_stick_y) < Math.abs(gamepad1.right_stick_x)) {
          telemetry.addData("It is working", 123);
          Motor1.setPower(gamepad1.right_stick_x * 1);
          Motor2.setPower(gamepad1.right_stick_x * 1);
          Motor3.setPower(gamepad1.right_stick_x * -1);
          Motor4.setPower(gamepad1.right_stick_x * -1);
        } else {
          Motor1.setPower(0);
          Motor2.setPower(0);
          Motor3.setPower(0);
          Motor4.setPower(0);
        }
        telemetry.addData("RightY", gamepad1.right_stick_y);
        telemetry.addData("RightX", gamepad1.right_stick_x);
        telemetry.update();
        }
    }
  }
}