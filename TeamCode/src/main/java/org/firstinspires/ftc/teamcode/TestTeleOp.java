package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.toolkit.core.UpliftTele;
import org.firstinspires.ftc.teamcode.toolkit.misc.UpliftMath;

import static java.lang.Math.PI;
import static java.lang.Math.abs;
import static java.lang.Math.sin;
import static java.lang.Math.sqrt;
import static java.lang.Math.toRadians;

@TeleOp (name = "TestTeleOp", group = "Opmodes")
public class TestTeleOp extends UpliftTele {

    UpliftRobot robot;
    @Override
    public void initHardware() {
        robot = new UpliftRobot(this);

    }

    @Override
    public void initAction() {
//

    }

    @Override
    public void bodyLoop() {
        double leftX = Range.clip(gamepad1.left_stick_x, -1, 1);
        double leftY = Range.clip(-gamepad1.left_stick_y, -1, 1);
        double rightX = Range.clip(gamepad1.right_stick_x, -1, 1);

        double angle = 90 - Math.toDegrees(UpliftMath.atan2UL(leftY, leftX));
        double magnitude = Range.clip(Math.sqrt(Math.pow(leftX, 2) + Math.pow(leftY, 2)), -1, 1);


        turn(90, 0.5, 0.5);
        robot.rawAngle = -robot.imu.getAngularOrientation().firstAngle;
        telemetry.addData("robot angle", robot.rawAngle);
        telemetry.update();



    }


    @Override
    public void exit() {

    }
    public static void teleDrive(double joystickAngle, double speedVal, double turnVal, UpliftRobot robot) {
        double lfPow = sin(toRadians(joystickAngle) + (0.25 * PI)) * speedVal + turnVal;
        double rfPow = sin(toRadians(joystickAngle) - (0.25 * PI)) * speedVal - turnVal;
        double lbPow = sin(toRadians(joystickAngle) - (0.25 * PI)) * speedVal + turnVal;
        double rbPow = sin(toRadians(joystickAngle) + (0.25 * PI)) * speedVal - turnVal;

        // find max total input out of the 4 motors
        double maxVal = abs(lfPow);
        if(abs(rfPow) > maxVal) {
            maxVal = abs(rfPow);
        }
        if(abs(lbPow) > maxVal) {
            maxVal = abs(lbPow);
        }
        if(abs(rbPow) > maxVal) {
            maxVal = abs(rbPow);
        }

        if(maxVal < (1 / sqrt(2))) {
            maxVal = 1 / sqrt(2);
        }

        // set the scaled powers
        robot.leftFront.setPower(lfPow / maxVal);
        robot.rightFront.setPower(rfPow / maxVal);
        robot.leftBack.setPower(lbPow / maxVal);
        robot.rightBack.setPower(rbPow / maxVal);
    }

    public void tankDrive(double leftPower, double rightPower, UpliftRobot robot) {
        robot.rightFront.setPower(rightPower);
        robot.leftFront.setPower(leftPower);
        robot.leftBack.setPower(leftPower);
        robot.rightBack.setPower(rightPower);
    }

    public void spin(double speed) {
        speed = Range.clip(speed, -1, 1);
        robot.leftFront.setPower(speed);
        robot.leftBack.setPower(speed);
        robot.rightFront.setPower(-speed);
        robot.rightBack.setPower(-speed);
    }

    public void turn(double degrees, double speed, double tolerance) {
        double initialAngle = robot.rawAngle;
        double targetAngle = initialAngle + degrees;
        // if turning counter-clockwise
        while(Math.abs(degrees) > tolerance) {
            if(degrees > 30) {
                spin(speed);
            } else if(degrees > 10) {
                spin(0.4);
            } else if(degrees > 5) {
                spin(0.2);
            } else if(degrees > 0) {
                spin(0.15);
            } else if(degrees < -30) {
                spin(-speed);
            } else if(degrees < -10) {
                spin(-0.4);
            } else if(degrees < -5) {
                spin(-0.2);
            } else if(degrees < 0) {
                spin(-0.15);
            } else {
                stopMotors();
                return;
            }
            degrees = targetAngle - robot.rawAngle;
        }
        stopMotors();
    }
    public void stopMotors() {
        robot.leftFront.setPower(0);
        robot.leftBack.setPower(0);
        robot.rightBack.setPower(0);
        robot.rightFront.setPower(0);
    }

}
