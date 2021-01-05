/**
 * This is the test program for Walrus.
 *
 * @author Sai
 * @version 1.0
 * @since 1/4/2020
 * @status finished
 */

//TODO Sai: add other motors and servos once they are connected

package org.firstinspires.ftc.teamcode.Autonomous.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "WalrusTest ", group="Tests: ")
//@Disabled
public class WalrusTest extends LinearOpMode {

    @Override
    public void runOpMode() {

        DcMotorEx motorLeftFront;
        DcMotorEx motorRightFront;
        DcMotorEx motorLeftBack;
        DcMotorEx motorRightBack;
        DcMotorEx intakeMotor;
        DcMotorEx shooterMotor;


        motorLeftFront = (DcMotorEx) hardwareMap.dcMotor.get("motorLeftFront");
        motorRightFront = (DcMotorEx) hardwareMap.dcMotor.get("motorRightFront");
        motorLeftBack = (DcMotorEx) hardwareMap.dcMotor.get("motorLeftBack");
        motorRightBack = (DcMotorEx) hardwareMap.dcMotor.get("motorRightBack");
        intakeMotor = (DcMotorEx) hardwareMap.dcMotor.get("IntakeMotor");
        shooterMotor = (DcMotorEx) hardwareMap.dcMotor.get("ShooterMotor");


        // All Motors
        motorLeftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorLeftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        motorLeftFront.setDirection(DcMotor.Direction.FORWARD);
        motorRightFront.setDirection(DcMotor.Direction.FORWARD);
        motorLeftBack.setDirection(DcMotor.Direction.FORWARD);
        motorRightBack.setDirection(DcMotor.Direction.FORWARD);
        intakeMotor.setDirection(DcMotor.Direction.FORWARD);
        shooterMotor.setDirection(DcMotor.Direction.FORWARD);

        // reset encoder count kept by the motors.
        motorLeftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorLeftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorRightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorRightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        ElapsedTime runtime = new ElapsedTime();
        //Robot robot = new Robot(this);

        waitForStart();
        //Tests all of the motors individually
        motorLeftFront.setPower(0.5);
        telemetry.addData("Moving LeftFront Motor", 0.5);
        telemetry.update();
        sleep(1000);
        motorLeftFront.setPower(0);
        sleep(1000);
        motorLeftFront.setPower(-0.5);
        telemetry.addData("Moving LeftFront Motor", -0.5);
        telemetry.update();
        sleep(1000);
        motorLeftFront.setPower(0);
        sleep(1000);
        motorRightFront.setPower(0.5);
        telemetry.addData("Moving RightFront Motor", 0.5);
        telemetry.update();
        sleep(1000);
        motorRightFront.setPower(0);
        sleep(1000);
        motorRightFront.setPower(-0.5);
        telemetry.addData("Moving RightFront Motor", -0.5);
        telemetry.update();
        sleep(1000);
        motorRightFront.setPower(0);
        sleep(1000);
        motorLeftBack.setPower(0.5);
        telemetry.addData("Moving LeftBack Motor", 0.5);
        telemetry.update();
        sleep(1000);
        motorLeftBack.setPower(0);
        sleep(1000);
        motorLeftBack.setPower(-0.5);
        telemetry.addData("Moving LeftBack Motor", -0.5);
        telemetry.update();
        sleep(1000);
        motorLeftBack.setPower(0);
        sleep(1000);
        motorRightBack.setPower(0.5);
        telemetry.addData("Moving RightBack Motor", 0.5);
        telemetry.update();
        sleep(1000);
        motorRightBack.setPower(0);
        sleep(1000);
        motorRightBack.setPower(-0.5);
        telemetry.addData("Moving RightBack Motor", -0.5);
        telemetry.update();
        sleep(1000);
        motorRightBack.setPower(0);
        sleep(1000);

        //Tests intake motor
        //Forwards
        intakeMotor.setPower(0.3);
        telemetry.addData("Moving intakeMotor forwards", 0.3);
        telemetry.update();
        sleep(1000);
        intakeMotor.setPower(0);
        sleep(1000);
        //Backwards
        intakeMotor.setPower(-0.3);
        telemetry.addData("Moving intakeMotor backwards", -0.3);
        telemetry.update();
        sleep(1000);
        intakeMotor.setPower(0);

        //Tests shooter motor but the motor has not been connected, so it is commented
//        shooterMotor.setPower(0.8);
//        telemetry.addData("Moving shooterMotor forwards", 0.8);
//        telemetry.update();
//        sleep(700);
//        shooterMotor.setPower(0);
//        sleep(1000);
//        shooterMotor.setPower(-0.8);
//        telemetry.addData("Moving shooterMotor backwards", -0.8);
//        telemetry.update();
//        sleep(700);
//        shooterMotor.setPower(0);
//        sleep(1000);
    }
}