package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@TeleOp(name="Pinpoint Drive Test", group="Test")
public class PinpointDriveTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        RockinBot robot = new RockinBot(this);

        telemetry.addData("Pinpoint Drive Test", "Ready");
        telemetry.addData("Coordinates", "X forward, Y left, heading counterclockwise");
        telemetry.addData("Reset pose", "Press gamepad 1 Options");
        telemetry.addData("Version", "v20261013-03");
        telemetry.update();
        waitForStart();

        boolean previousOptions = false;
        while (opModeIsActive()) {
            robot.setWheelPower(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    gamepad1.right_stick_x);

            if (gamepad1.options && !previousOptions) {
                robot.odo.resetPosAndIMU();
            }
            previousOptions = gamepad1.options;

            robot.odo.update();
            Pose2D pose = robot.odo.getPosition();
            telemetry.addData("X (mm)", "%.1f", pose.getX(DistanceUnit.MM));
            telemetry.addData("Y (mm)", "%.1f", pose.getY(DistanceUnit.MM));
            telemetry.addData("Heading (degrees)", "%.1f", pose.getHeading(AngleUnit.DEGREES));
            telemetry.addData("Pinpoint status", robot.odo.getDeviceStatus());
            telemetry.addData("Reset pose", "Press gamepad 1 Options");
            telemetry.update();
            idle();
        }
    }
}
