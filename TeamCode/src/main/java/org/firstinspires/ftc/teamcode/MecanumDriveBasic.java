package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;


@TeleOp(name = "MecanumDriveBasic")
public class MecanumDriveBasic extends LinearOpMode {
  @Override
  public void runOpMode() {

    /* we keep track of how long it's been since the OpMode was started, just
     * to have some interesting data to show */
    ElapsedTime opmodeRunTime = new ElapsedTime();

    // We show the log in oldest-to-newest order, as that's better for poetry
    telemetry.log().setDisplayOrder(Telemetry.Log.DisplayOrder.OLDEST_FIRST);
    // We can control the number of lines shown in the log
    telemetry.log().setCapacity(6);

    // Declare our motors
    // Make sure your ID's match your configuration
    DcMotor front_right_motor = hardwareMap.dcMotor.get("front_right_motor");
    DcMotor rear_right_motor = hardwareMap.dcMotor.get("rear_right_motor");
    DcMotor front_left_motor = hardwareMap.dcMotor.get("front_left_motor");
    DcMotor rear_left_motor = hardwareMap.dcMotor.get("rear_left_motor");

    // Reverse the right side motors.  This may be wrong for your setup.
    // If your robot moves backwards when commanded to go forwards, reverse the left side instead.
    front_right_motor.setDirection(DcMotorSimple.Direction.REVERSE);
    rear_right_motor.setDirection(DcMotorSimple.Direction.REVERSE);

    waitForStart();

    telemetry.addData("time", "%.1f seconds", opmodeRunTime.seconds());
    telemetry.update();
    idle(); // Take a break until opModeIsActive.

    while (opModeIsActive()) {
      double drive = gamepad1.left_stick_y;
      double strafe = gamepad1.left_stick_x;
      double rotate = gamepad1.right_stick_x;
      // Denominator is the largest motor power (absolute value) or 1.
      // This ensures all powers maintain the same ratio, but only if one is outside of the range [-1, 1].
      // This should work better than simply clipping the power.
      double max = Math.max(Math.abs(drive) + Math.abs(strafe) + Math.abs(rotate),1);
      //the next line is from blocks generated java for compar
      //max = JavaUtil.maxOfList(JavaUtil.createListWith(JavaUtil.sumOfList(JavaUtil.createListWith(Math.abs(drive), Math.abs(strafe), Math.abs(rotate))), 1));
      double frontLeftPower = (drive + strafe + rotate) / max;
      double rearLeftPower = (drive - strafe + rotate) / max;
      double frontRightPower = (drive - strafe - rotate) / max;
      double rearRightPower = (drive + strafe - rotate) / max;

      front_left_motor.setPower(frontLeftPower);
      rear_left_motor.setPower(rearLeftPower);
      front_right_motor.setPower(frontRightPower);
      rear_right_motor.setPower(rearRightPower);
    }
  }
}