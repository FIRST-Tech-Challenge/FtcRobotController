package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareDeviceCloseOnTearDown;

import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.Navigation.Odometry;

@TeleOp
public class TestTeleOp extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        Robot robot = new Robot(hardwareMap, this, telemetry, false, false, false);
        Odometry odometry = new Odometry(robot.backEncoder, robot.rightEncoder, robot.leftEncoder, robot.opMode,
                telemetry, 0, 0, 0);
        //robot.initForTeleOp();
        robot.setUpDrivetrainMotors();
        int TRIGGER_PRESSED = 0; // TODO: test
        int frontFacing = 1;
        boolean slowMode = false;

        //doubles for amount of input for straight, turning, and mecanuming variables
        double straight;
        double turning;
        double mecanuming;

        double fLeftPower;
        double fRightPower;
        double bLeftPower;
        double bRightPower;
        double maxPower;
        double scale;

        double fLeftPowerPrev = 0;
        double fRightPowerPrev = 0;
        double bLeftPowerPrev = 0;
        double bRightPowerPrev = 0;
        waitForStart();

        while (opModeIsActive()) {
            straight = (gamepad1.left_stick_y)*(gamepad1.left_stick_y)*(gamepad1.left_stick_y) * frontFacing * -1;
            mecanuming = (gamepad1.left_stick_x)*(gamepad1.left_stick_x)*(gamepad1.left_stick_x) * frontFacing;

            //turning stays the same
            turning = (gamepad1.right_stick_x) * (gamepad1.right_stick_x) * (gamepad1.right_stick_x);

            //Pure Mecanum overrides straight and turn
            if (gamepad1.right_trigger != 0) {
                straight = 0;
                turning = 0;
                mecanuming = 0.7;
            } else if (gamepad1.left_trigger != 0) {
                straight = 0;
                turning = 0;
                mecanuming = -0.7;
            }

            //set powers using this input
            fLeftPower = straight + turning + mecanuming;
            fRightPower = straight - turning - mecanuming;
            bLeftPower = straight + turning - mecanuming;
            bRightPower = straight - turning + mecanuming;


            //scale powers
            maxPower = robot.maxAbsValueDouble(fLeftPower, bLeftPower, fRightPower, bRightPower);

            if (Math.abs(maxPower) > 1) {
                scale = Math.abs(maxPower);
                fLeftPower /= scale;
                bLeftPower /= scale;
                fRightPower /= scale;
                bRightPower /= scale;
            }

            //uses different powers based on which bumper was pressed last
            if (slowMode) {
                fLeftPower *= 0.7;
                bLeftPower *= 0.7;
                fRightPower *= 0.7;
                bRightPower *= 0.7;
            }

            //set motor power ONLY if a value has changed. else, use previous value.
            if (fLeftPowerPrev != fLeftPower || fRightPowerPrev != fRightPower
                    || bLeftPowerPrev != bLeftPower || bRightPowerPrev != bRightPower) {
                robot.drivetrain.setMotorPower(fLeftPower, fRightPower, bLeftPower, bRightPower);

                fLeftPowerPrev = fLeftPower;
                fRightPowerPrev = fRightPower;
                bLeftPowerPrev = bLeftPower;
                bRightPowerPrev = bRightPower;
            }

            odometry.updatePosition();

        }
    }
}
