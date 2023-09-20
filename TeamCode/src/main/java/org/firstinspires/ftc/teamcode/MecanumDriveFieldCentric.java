package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


@TeleOp(name = "MecanumDriveFieldCentric")
public class MecanumDriveFieldCentric extends LinearOpMode {
  @Override
  public void runOpMode() {
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

    // Retrieve the IMU from the hardware map
    IMU imu = hardwareMap.get(IMU.class, "imu");
    // ***Adjust the orientation parameters to match your robot***
    IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
            RevHubOrientationOnRobot.LogoFacingDirection.UP,
            RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
    // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
    imu.initialize(parameters);

    waitForStart();

    if (isStopRequested()) return;

    while (opModeIsActive()) {

      double drive = gamepad1.left_stick_y * -1;// Remember, Y stick value is reversed
      double strafe = gamepad1.left_stick_x * 1.1;// Factor to counteract imperfect strafing
      double rotate = gamepad1.right_stick_x;

      // This button choice was made so that it is hard to hit on accident,
      // it can be freely changed based on preference.
      // The equivalent button is start on Xbox-style controllers.
      if (gamepad1.options) {
        imu.resetYaw();
      }

      double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

      // Rotate the movement direction counter to the bot's rotation
      double rotX = strafe * Math.cos(-botHeading) - drive * Math.sin(-botHeading);
      double rotY = strafe * Math.sin(-botHeading) + drive * Math.cos(-botHeading);

      rotX = rotX * 1.1;  // Counteract imperfect strafing

      // Denominator is the largest motor power (absolute value) or 1.
      // This ensures all powers maintain the same ratio, but only if one is outside of the range [-1, 1].
      // This should work better than simply clipping the power.
      double denominator = Math.max(Math.abs(drive) + Math.abs(strafe) + Math.abs(rotate), 1);
      //denominator = JavaUtil.maxOfList(JavaUtil.createListWith(JavaUtil.sumOfList(JavaUtil.createListWith(Math.abs(drive), Math.abs(strafe), Math.abs(rotate))), 1)); //this line is from blocks generated java
      double frontLeftPower = (rotY + rotX + rotate) / denominator;
      double rearLeftPower = (rotY - rotX + rotate) / denominator;
      double frontRightPower = (rotY - rotX - rotate) / denominator;
      double rearRightPower = (rotY + rotX - rotate) / denominator;

      front_left_motor.setPower(frontLeftPower);
      rear_left_motor.setPower(rearLeftPower);
      front_right_motor.setPower(frontRightPower);
      rear_right_motor.setPower(rearRightPower);
    }
  }
}
