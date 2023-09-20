package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.subsystems.RobotHardware;

@TeleOp(name = "MecanumDrive_HwClass")
public class MecanumDrive_HwClass extends LinearOpMode {
  RobotHardware drive = new RobotHardware(); //*** This was the trick**
  @Override
  public void runOpMode() {
    //RobotHardware.init(hardwareMap); //This did not work
    drive.init(hardwareMap);

    waitForStart();

    idle(); // Take a break until opModeIsActive.

    while (opModeIsActive()) {

      double y = gamepad1.left_stick_y;
      double x = gamepad1.left_stick_x;
      double rx = gamepad1.right_stick_x;

      drive.drive(y, x, rx);
    }
  }
}
