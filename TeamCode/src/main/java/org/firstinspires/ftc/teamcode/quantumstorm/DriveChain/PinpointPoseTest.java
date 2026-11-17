package org.firstinspires.ftc.teamcode.quantumstorm.DriveChain;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.quantumstorm.Constants;
import org.firstinspires.ftc.teamcode.quantumstorm.Sensors.PinpointOdometry;

/**
 * Use this to measure field locations for Constants (RED_GARDEN_POSE, RED_1_HIVE1_POSE, ...).
 * 1. Place the robot at RED_1_START_POSE and press INIT (robot must be still).
 * 2. Drive or push the robot to a location and read X / Y / Heading.
 * Also checks pod directions: forward should increase X, strafing left should increase Y.
 */
@Disabled
@TeleOp(name = "Test: Pinpoint Pose", group = "Quantum Storm Test")
public class PinpointPoseTest extends LinearOpMode {

    @Override
    public void runOpMode() {

        MecanumDrive drive = new MecanumDrive(hardwareMap);
        PinpointOdometry odometry = new PinpointOdometry(hardwareMap, Constants.RED_1_START_POSE);

        telemetry.addLine("Place robot at Red Team One start, then press PLAY");
        telemetry.addLine("Sticks drive, B = reset pose to Red Team One start");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            odometry.update();

            if (gamepad1.b) {
                odometry.setPose(Constants.RED_1_START_POSE);
            }

            drive.drive(
                    -gamepad1.left_stick_y,
                    gamepad1.left_stick_x,
                    gamepad1.right_stick_x);

            odometry.addTelemetry(telemetry);
            telemetry.update();
        }

        drive.stop();
    }
}
