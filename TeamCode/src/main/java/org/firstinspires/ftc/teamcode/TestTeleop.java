
package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp

public class TestTeleop extends LinearOpMode {

    robotHardware robot = new robotHardware();

    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap);
        waitForStart();

        MecanumDrive mecanumDrive = new MecanumDrive(robot.leftFront, robot.rightFront, robot.leftBack, robot.rightBack);
        GamepadEx gamePad = new GamepadEx(gamepad1);

        // Tune to driver preference
        double driveGain = 0.5;
        double turnGain = 0.5;

        double xSpeed;
        double ySpeed;
        double turnSpeed;
        double gyroAngle;

        while (opModeIsActive()) {
            xSpeed = ether(gamePad.getLeftX(), driveGain);
            ySpeed = ether(gamePad.getLeftY(), driveGain);
            turnSpeed = ether(gamePad.getRightX(), turnGain);
            gyroAngle = Math.toDegrees(robot.revIMU.getHeading() * Math.PI + Math.PI);

            mecanumDrive.driveFieldCentric(xSpeed, ySpeed, turnSpeed, gyroAngle, true);
        }

    }

    // https://www.desmos.com/calculator/dx7yql2ekh
    public static double ether(double x, double p) {
        double min = 0.2;
        double max = 1;

        if (x > 0) {
            min = 0 - min;
        }
        return min + (1 - min) * (p * Math.pow(x, 3) + (1 - p) * x);
    }
}
