package org.firstinspires.ftc.teamcode.quantumstorm.DriveChain;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.quantumstorm.Constants;

/**
 * Drive chain check-out OpMode.
 * Hold a button to spin ONE wheel forward, to verify wiring, port names and
 * motor directions. With no button held, the sticks drive the robot normally.
 */
@Disabled
@TeleOp(name = "Test: Mecanum Drive", group = "Quantum Storm Test")
public class MecanumDriveTest extends LinearOpMode {

    @Override
    public void runOpMode() {

        MecanumDrive drive = new MecanumDrive(hardwareMap);

        telemetry.addLine("Mecanum Drive Test Ready");
        telemetry.addLine("");
        telemetry.addLine("Hold X = Front Left wheel");
        telemetry.addLine("Hold Y = Front Right wheel");
        telemetry.addLine("Hold A = Back Left wheel");
        telemetry.addLine("Hold B = Back Right wheel");
        telemetry.addLine("Each wheel should spin FORWARD");
        telemetry.addLine("");
        telemetry.addLine("No button = normal stick drive");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            double p = Constants.DRIVE_TEST_POWER;
            String mode;

            if (gamepad1.x) {
                drive.setPowers(p, 0, 0, 0);
                mode = "Front Left only";
            } else if (gamepad1.y) {
                drive.setPowers(0, p, 0, 0);
                mode = "Front Right only";
            } else if (gamepad1.a) {
                drive.setPowers(0, 0, p, 0);
                mode = "Back Left only";
            } else if (gamepad1.b) {
                drive.setPowers(0, 0, 0, p);
                mode = "Back Right only";
            } else {
                drive.drive(
                        -gamepad1.left_stick_y,
                        gamepad1.left_stick_x,
                        gamepad1.right_stick_x);
                mode = "Stick drive";
            }

            telemetry.addData("Mode", mode);
            drive.addTelemetry(telemetry);
            telemetry.update();
        }

        drive.stop();
    }
}
