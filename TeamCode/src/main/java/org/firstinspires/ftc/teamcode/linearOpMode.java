package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import static java.lang.Math.sqrt;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
//import org.firstinspires.ftc.teamcode.components.device.GamepadKt;
//import org.firstinspires.ftc.teamcode.components.meta.Hardware;
//import org.firstinspires.ftc.teamcode.components.meta.MotorGroup;

@TeleOp(name = "DriveTrainTest1 (Blocks to Java)")
public class linearOpMode extends LinearOpMode {

  private DcMotor leftBack;
  private DcMotor leftFront;
  private DcMotor rightBack;
  private DcMotor rightFront;
  private Servo claw;

  /**
   * This sample contains the bare minimum Blocks for any regular OpMode. The 3 blue
   * Comment Blocks show where to place Initialization code (runs once, after touching the
   * DS INIT button, and before touching the DS Start arrow), Run code (runs once, after
   * touching Start), and Loop code (runs repeatedly while the OpMode is active, namely not
   * Stopped).
   */
  @Override
  public void runOpMode() {
    leftBack = hardwareMap.get(DcMotor.class, "leftBack");
    leftFront = hardwareMap.get(DcMotor.class, "leftFront");
    rightBack = hardwareMap.get(DcMotor.class, "rightBack");
    rightFront = hardwareMap.get(DcMotor.class, "rightFront");
    claw = hardwareMap.get(Servo.class, "claw");

    // Put initialization blocks here.
    waitForStart();
    if (opModeIsActive()) {
      // Put run blocks here.
      while (opModeIsActive()) {
//        leftBack.setPower(-gamepad1.left_stick_y);
//        leftFront.setPower(-gamepad1.left_stick_y);
//        rightBack.setPower(gamepad1.right_stick_y);
//        rightFront.setPower(gamepad1.right_stick_y);
//        telemetry.update();

          drive(gamepad1, 1);
      }
    }
  }

  // drive code stolen from Anthony
  public void drive(Gamepad gamepad, double powerMulti) {
    double[] driveSticks = GamepadKt.getDriveSticks(gamepad);
    double x = driveSticks[0];
    double y = driveSticks[1];
    double r = driveSticks[2];

    double theta = Math.atan2(y, x);
    double power = Math.min(Math.hypot(x, y), 1.0);

    double xComponent = power * Math.cos(theta - Math.PI / 4);
    double yComponent = power * Math.sin(theta - Math.PI / 4);
    double[] arr = {Math.abs(xComponent), Math.abs(yComponent), Math.abs(1e-16)}; // ? untested
    double max = largest(arr);

    double[] powers = {
            power * (xComponent / max) + r,
            power * (yComponent / max) - r,
            power * (yComponent / max) + r,
            power * (xComponent / max) - r
    };


//        if (power + Math.abs(r) > 1) {
//            for (int i = 0; i < powers.length; i++) {
//                powers[i] /= (power + Math.abs(r));
//            }
//        }

    double _powerMulti = !GamepadKt.isAnyJoystickTriggered(gamepad) ? 0.0 : powerMulti;

    for (int i = 0; i < powers.length; i++) {
      powers[i] = Math.pow(powers[i], 3.0) * _powerMulti;
    }

    //powers = reduce(powers);

    double[] finalPowers = powers;
    motorGroup.applyToMotors(motor -> motor.setPower(finalPowers[motorGroup.motorIndex]));
//        telemetry.addData("FL", frontLeft.getPower());
//        telemetry.addData("FR", frontRight.getPower());
//        telemetry.addData("BL", backLeft.getPower());
//        telemetry.addData("BR", backRight.getPower());
//        telemetry.addData("FLF", powers[0]);
//        telemetry.addData("FRF", powers[1]);
//        telemetry.addData("BLF", powers[2]);
//        telemetry.addData("BRF", powers[3]);
//        telemetry.update();

  }

}
