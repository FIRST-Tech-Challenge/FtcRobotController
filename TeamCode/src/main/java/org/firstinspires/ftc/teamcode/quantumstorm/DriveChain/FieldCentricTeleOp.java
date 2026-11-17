package org.firstinspires.ftc.teamcode.quantumstorm.DriveChain;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.quantumstorm.Constants;

/**
 * Drive-only field-centric TeleOp for the Starter Bot (4-motor mecanum + Control Hub IMU).
 * Pushing the left stick away from the driver always drives away from the driver,
 * whichever way the robot faces. Needs only the four drive motors and "imu".
 *
 * Start each match with the robot facing AWAY from the driver; INIT sets that as 0.
 */
@TeleOp(name = "Starter Bot Field-Centric", group = "Quantum Storm")
public class FieldCentricTeleOp extends LinearOpMode {

    @Override
    public void runOpMode() {

        MecanumDrive drive = new MecanumDrive(hardwareMap);

        IMU imu = hardwareMap.get(IMU.class, Constants.IMU);
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                Constants.HUB_LOGO_DIRECTION,
                Constants.HUB_USB_DIRECTION)));

        // Whatever way the robot faces now becomes "away from the driver"
        imu.resetYaw();

        telemetry.addLine("Starter Bot Field-Centric Ready");
        telemetry.addLine("Point the robot AWAY from the driver, then press START");
        telemetry.addLine("");
        telemetry.addLine("Left Stick = Move (relative to the driver)");
        telemetry.addLine("Right Stick X = Turn");
        telemetry.addLine("Hold RB = Slow mode");
        telemetry.addLine("Y = Reset heading (robot must face away from driver)");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // Y = robot is facing away from the driver right now
            if (gamepad1.y) {
                imu.resetYaw();
            }

            double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            // Stick Y is negative when pushed forward
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            if (gamepad1.right_bumper) {
                y *= Constants.FIELD_CENTRIC_SLOW_SCALE;
                x *= Constants.FIELD_CENTRIC_SLOW_SCALE;
                rx *= Constants.FIELD_CENTRIC_SLOW_SCALE;
            }

            drive.driveFieldCentric(y, x, rx, heading);

            telemetry.addData("Heading", "%.1f deg (0 = away from driver, + = turned left)",
                    Math.toDegrees(heading));
            telemetry.addData("Speed", gamepad1.right_bumper ? "SLOW" : "FULL");
            drive.addTelemetry(telemetry);
            telemetry.update();
        }

        drive.stop();
    }
}
