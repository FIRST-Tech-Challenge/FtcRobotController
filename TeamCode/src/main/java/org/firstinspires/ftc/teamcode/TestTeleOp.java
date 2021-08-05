package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
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
    DcMotor rf, lf, rb, lb;
    Servo claw;
    Servo wobble1, wobble2;
    DcMotor intake;
    Servo intakeLifter;
    DcMotor transfer;
    DcMotorEx shooter1;
    DcMotorEx shooter2;
    Servo flicker;
    AnalogInput potentiometer;


    @Override
    public void initHardware() {
        robot = new UpliftRobot(this);
        rf = robot.rightFront;
        lf = robot.leftFront;
        rb = robot.rightBack;
        lb = robot.leftBack;
        claw = robot.clamp;
        wobble1 = robot.wobbleLeft;
        wobble2 = robot.wobbleRight;
        intake = robot.intake;
        intakeLifter = robot.intakeLifter;
        transfer = robot.transfer;
        shooter1 = robot.shooter1;
        shooter2 = robot.shooter2;
        flicker = robot.flicker;
    }

    @Override
    public void initAction() {
//        claw.setPosition(0.5);

    }

    @Override
    public void bodyLoop() {
        double leftX = Range.clip(gamepad1.left_stick_x, -1, 1);
        double leftY = Range.clip(-gamepad1.left_stick_y, -1, 1);
        double rightX = Range.clip(gamepad1.right_stick_x, -1, 1);

        double angle = 90 - Math.toDegrees(UpliftMath.atan2UL(leftY, leftX));
        double magnitude = Range.clip(Math.sqrt(Math.pow(leftX, 2) + Math.pow(leftY, 2)), -1, 1);

        teleDrive(angle, magnitude, rightX, robot);

        if(gamepad1.right_bumper) {
            pickUp();
        }
        if(gamepad1.left_bumper) {
            drop();
        }
        if(gamepad1.a) {
            halfDrop();
        }
        intake.setPower(-gamepad2.left_stick_y);
        if(gamepad2.a) {
            liftRoller();
        }
        if(gamepad2.b) {
            dropRoller();
        }
        transfer.setPower(gamepad2.right_stick_y / 2);
        if(gamepad2.x) {
            shooter1.setPower(1);
            shooter2.setPower(1);
        }
        if(gamepad2.y) {
            shooter1.setPower(0);
            shooter2.setPower(0);
        }
        if(gamepad2.right_bumper) {
            Log.i("RB clicked", "TRUE");
            for(int i = 0; i < 3; i++) {
                setFlickerIn();
                Log.i("FLICKER IN", "TRUE");
                robot.safeSleep(300);
                setFlickerOut();
                robot.safeSleep(300);
            }
        }


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
    public void wobbleUp() {
        wobble1.setPosition(0.2);
        wobble2.setPosition(0.8);

    }
    public void wobbleDown() {
        wobble1.setPosition(1);
        wobble2.setPosition(0);
    }
    public void clampOpen() {
        claw.setPosition(0.3);
    }
    public void clampClose() {
        claw.setPosition(0);

    }
    public void pickUp() {
        clampClose();
        robot.safeSleep(1000);
        wobbleUp();
    }
    public void drop() {
        wobbleDown();
        robot.safeSleep(1000);
        clampOpen();
    }
    public void halfDrop() {
        wobble1.setPosition(0.5);
        wobble2.setPosition(0.5);
        clampOpen();
    }
    public void liftRoller() {
        intakeLifter.setPosition(0.82);
    }
    public void dropRoller() {
        intakeLifter.setPosition(0.64);
    }

    public void setFlickerOut() {
        flicker.setPosition(0.49);
    }

    public void setFlickerIn() {
        flicker.setPosition(0.618);
    }

    public void flickRing() {
        // move in
        setFlickerIn();
        double initialTime = System.currentTimeMillis();
        while(potentiometer.getVoltage() > 0.635 && System.currentTimeMillis() - initialTime < 750) {
            robot.safeSleep(1);
        }
        setFlickerOut();
        initialTime = System.currentTimeMillis();
        while(potentiometer.getVoltage() < 0.99 && System.currentTimeMillis() - initialTime < 750) {
            robot.safeSleep(1);
        }
    }



}
