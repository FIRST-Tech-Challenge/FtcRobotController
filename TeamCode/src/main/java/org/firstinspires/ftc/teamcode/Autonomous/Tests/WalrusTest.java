/**
 * This is the test program for Walrus.
 *
 * @author Sai
 * @version 1.0
 * @since 1/4/2020
 * @status finished
 */

//TODO Sai: fix the wobbleGoalServoClaw (it is not moving right now)
//TODO Sai: add the wobbleGoalServo once it is added onto the robot

package org.firstinspires.ftc.teamcode.Autonomous.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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
        Servo shooter_Servo;
        DcMotorEx wobbleGoalMotor;
        Servo wobbleGoalServo;
        Servo wobbleGoalServoClaw;


        motorLeftFront = (DcMotorEx) hardwareMap.dcMotor.get("motorLeftFront");
        motorRightFront = (DcMotorEx) hardwareMap.dcMotor.get("motorRightFront");
        motorLeftBack = (DcMotorEx) hardwareMap.dcMotor.get("motorLeftBack");
        motorRightBack = (DcMotorEx) hardwareMap.dcMotor.get("motorRightBack");
        intakeMotor = (DcMotorEx) hardwareMap.dcMotor.get("IntakeMotor");
        shooterMotor = (DcMotorEx) hardwareMap.dcMotor.get("ShooterMotor");
        wobbleGoalMotor = (DcMotorEx) hardwareMap.dcMotor.get("wobbleGoalMotor");
        shooter_Servo = (Servo) hardwareMap.servo.get("ShooterServo");
        wobbleGoalServo = (Servo) hardwareMap.servo.get("WobbleGoalServo");
        wobbleGoalServoClaw = (Servo) hardwareMap.servo.get("wobbleGoalServoClaw");


        // All Motors
        motorLeftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorLeftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        wobbleGoalMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Servos
        shooter_Servo.setPosition(-0.5);
        wobbleGoalServo.setPosition(0);
        wobbleGoalServoClaw.setPosition(0);

        // To match default
        motorLeftFront.setDirection(DcMotor.Direction.REVERSE);
        motorRightFront.setDirection(DcMotor.Direction.FORWARD);
        motorLeftBack.setDirection(DcMotor.Direction.REVERSE);
        motorRightBack.setDirection(DcMotor.Direction.FORWARD);
        intakeMotor.setDirection(DcMotor.Direction.FORWARD);
        shooterMotor.setDirection(DcMotor.Direction.FORWARD);
        wobbleGoalMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        // make the motors run on only power
        motorLeftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorLeftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorRightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorRightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        wobbleGoalMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

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

        //Tests shooter motor
        shooterMotor.setPower(0.8);
        telemetry.addData("Moving shooterMotor forwards", 0.8);
        telemetry.update();
        sleep(700);
        shooterMotor.setPower(0);
        sleep(1000);
        shooterMotor.setPower(-0.8);
        telemetry.addData("Moving shooterMotor backwards", -0.8);
        telemetry.update();
        sleep(700);
        shooterMotor.setPower(0);
        sleep(1000);

        //Tests Shooter Servo
        shooter_Servo.setPosition(0.5);
        telemetry.addData("Moving Shooter Servo", 0.5);
        telemetry.update();
        sleep(1000);
        shooter_Servo.setPosition(0);
        sleep(2000);
        shooter_Servo.setPosition(-0.5);
        telemetry.addData("Moving Shooter Servo", -0.5);
        telemetry.update();
        sleep(1000);
        shooter_Servo.setPosition(0);

        //Moving wobbleGoalMotor
        wobbleGoalMotor.setPower(0.25);
        telemetry.addData("Moving wobbleGoalMotor", 0.25);
        telemetry.update();
        sleep(750);
        wobbleGoalMotor.setPower(0);
        sleep(1000);
        wobbleGoalMotor.setPower(-0.3);
        telemetry.addData("Moving wobbleGoalMotor", -0.25);
        telemetry.update();
        sleep(750);
        wobbleGoalMotor.setPower(0);
        sleep(1500);

        //Wobble Goal Servo Claw
        wobbleGoalServoClaw.setPosition(1);
        telemetry.addData("Moving wobbleGoalServoClaw", 1);
        telemetry.update();
        sleep(1000);
        wobbleGoalServoClaw.setPosition(0);
        telemetry.addData("Moving wobbleGoalServoClaw", 0);
        telemetry.update();
        sleep(1500);

        //Wobble Goal Servo
//        wobbleGoalServo.setPosition(1);
//        sleep(1000);
//        wobbleGoalServo.setPosition(0);
//        sleep(2000);
    }
}