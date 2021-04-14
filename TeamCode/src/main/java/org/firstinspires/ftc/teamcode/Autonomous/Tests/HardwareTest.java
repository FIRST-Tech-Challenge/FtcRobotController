/**
 * This is the test program for Walrus.
 *
 * This is the wiring configuration of Walrus: https://docs.google.com/document/d/1umaQxQ1bZnItleHeQlGetBS1Zwyi2ag-nbSXEFhcqCI/edit
 *
 * @author Sai
 * @version 1.0
 * @since 1/4/2020
 * @status finished
 */

package org.firstinspires.ftc.teamcode.Autonomous.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "HardwareTest ", group="Tests: ")
//@Disabled
public class HardwareTest extends LinearOpMode {

    @Override
    public void runOpMode() {

        DcMotorEx motorLeftFront;
        boolean motorLeftFrontConfigured = true;
        DcMotorEx motorRightFront;
        boolean motorRightFrontConfigured = true;
        DcMotorEx motorLeftBack;
        boolean motorLeftBackConfigured = true;
        DcMotorEx motorRightBack;
        boolean motorRightBackConfigured = true;
        DcMotorEx intakeMotor;
        boolean intakeMotorConfigured = true;
        DcMotorEx transferMotor;
        boolean transferMotorConfigured = true;
        DcMotorEx shooterMotor;
        boolean shooterMotorConfigured = true;
        Servo shooter_Servo;
        boolean shooter_ServoConfigured = true;
        DcMotorEx wobbleGoalMotor;
        boolean wobbleGoalMotorConfigured = true;
        Servo wobbleGoalServoClaw;
        boolean wobbleGoalServoClawConfigured = true;
        Servo webcamServo;
        boolean webcamServoConfigured = true;
        Servo leftStick;
        boolean leftStickConfigured = true;
        Servo rightStick;
        boolean rightStickConfigured = true;

        try {
            motorLeftFront = (DcMotorEx) hardwareMap.dcMotor.get("motorLeftFront");
        } catch (Exception e) {
             motorLeftFrontConfigured = false;
        }
        try {
            motorRightFront = (DcMotorEx) hardwareMap.dcMotor.get("motorRightFront");
        } catch (Exception e) {
            motorRightFrontConfigured = false;
        }
        try {
            motorLeftBack = (DcMotorEx) hardwareMap.dcMotor.get("motorLeftBack");
        } catch (Exception e) {
            motorLeftBackConfigured = false;
        }
        try {
            motorRightBack = (DcMotorEx) hardwareMap.dcMotor.get("motorRightBack");
        } catch (Exception e) {
            motorRightBackConfigured = false;
        }
        try {
            intakeMotor = (DcMotorEx) hardwareMap.dcMotor.get("IntakeMotor");
        } catch (Exception e) {
            intakeMotorConfigured = false;
        }
        try {
            transferMotor = (DcMotorEx) hardwareMap.dcMotor.get("TransferMotor");
        } catch (Exception e) {
            transferMotorConfigured = false;
        }
        try {
            shooterMotor = (DcMotorEx) hardwareMap.dcMotor.get("ShooterMotor");
        } catch (Exception e) {
            shooterMotorConfigured = false;
        }
        try {
            wobbleGoalMotor = (DcMotorEx) hardwareMap.dcMotor.get("wobbleGoalMotor");
        } catch (Exception e) {
            wobbleGoalMotorConfigured = false;
        }
        try {
            shooter_Servo = hardwareMap.servo.get("ShooterServo");
        } catch (Exception e) {
            shooter_ServoConfigured = false;
        }
        try {
            wobbleGoalServoClaw = hardwareMap.servo.get("wobbleGoalServoClaw");
        } catch (Exception e) {
            wobbleGoalServoClawConfigured = false;
        }
        try {
            webcamServo = hardwareMap.servo.get("TensorFlowServo");
        } catch (Exception e) {
            webcamServoConfigured = false;
        }
        try {
            leftStick = hardwareMap.servo.get("leftStick");
        } catch (Exception e) {
            leftStickConfigured = false;
        }
        try {
            rightStick = hardwareMap.servo.get("rightStick");
        } catch (Exception e) {
            rightStickConfigured = false;
        }

        if (wobbleGoalServoClawConfigured) {
            wobbleGoalServoClaw = hardwareMap.servo.get("wobbleGoalServoClaw");
            wobbleGoalServoClaw.setPosition(1);
        }
        if (webcamServoConfigured) {
            webcamServo = hardwareMap.servo.get("TensorFlowServo");
            webcamServo.setPosition(0.75);
        }
        if (leftStickConfigured) {
            leftStick = hardwareMap.servo.get("leftStick");
            leftStick.setPosition(1);
        }
        if (rightStickConfigured) {
            rightStick = hardwareMap.servo.get("rightStick");
            rightStick.setPosition(0);
        }
        waitForStart();

        //Motors
        if (motorLeftFrontConfigured) {
            motorLeftFront = (DcMotorEx) hardwareMap.dcMotor.get("motorLeftFront");
            motorLeftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motorLeftFront.setDirection(DcMotor.Direction.REVERSE);
            motorLeftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motorLeftFront.setPower(0.5);
            telemetry.addData("Moving LeftFront Motor", "forward");
            telemetry.update();
            sleep(1000);
            motorLeftFront.setPower(0);
            sleep(1000);
            motorLeftFront.setPower(-0.5);
            telemetry.addData("Moving LeftFront Motor", "backward");
            telemetry.update();
            sleep(1000);
            motorLeftFront.setPower(0);
            sleep(1000);
        } else {
            telemetry.addData("motorLeftFront", "not in configurations");
            telemetry.update();
            sleep(5000);
        }
        if (motorRightFrontConfigured) {
            motorRightFront = (DcMotorEx) hardwareMap.dcMotor.get("motorRightFront");
            motorRightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motorRightFront.setDirection(DcMotor.Direction.FORWARD);
            motorRightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motorRightFront.setPower(0.5);
            telemetry.addData("Moving RightFront Motor", "forward");
            telemetry.update();
            sleep(1000);
            motorRightFront.setPower(0);
            sleep(1000);
            motorRightFront.setPower(-0.5);
            telemetry.addData("Moving RightFront Motor", "backward");
            telemetry.update();
            sleep(1000);
            motorRightFront.setPower(0);
            sleep(1000);
        } else {
            telemetry.addData("motorRightFront", "not in configurations");
            telemetry.update();
            sleep(5000);
        }
        if (motorLeftBackConfigured) {
            motorLeftBack = (DcMotorEx) hardwareMap.dcMotor.get("motorLeftBack");
            motorLeftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motorLeftBack.setDirection(DcMotor.Direction.REVERSE);
            motorLeftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motorLeftBack.setPower(0.5);
            telemetry.addData("Moving LeftBack Motor", "forward");
            telemetry.update();
            sleep(1000);
            motorLeftBack.setPower(0);
            sleep(1000);
            motorLeftBack.setPower(-0.5);
            telemetry.addData("Moving LeftBack Motor", "backward");
            telemetry.update();
            sleep(1000);
            motorLeftBack.setPower(0);
            sleep(1000);
        } else {
            telemetry.addData("motorLeftBack", "not in configurations");
            telemetry.update();
            sleep(5000);
        }
        if (motorRightBackConfigured) {
            motorRightBack = (DcMotorEx) hardwareMap.dcMotor.get("motorRightBack");
            motorRightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motorRightBack.setDirection(DcMotor.Direction.FORWARD);
            motorRightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motorRightBack.setPower(0.5);
            telemetry.addData("Moving RightBack Motor", "forward");
            telemetry.update();
            sleep(1000);
            motorRightBack.setPower(0);
            sleep(1000);
            motorRightBack.setPower(-0.5);
            telemetry.addData("Moving RightBack Motor", "backward");
            telemetry.update();
            sleep(1000);
            motorRightBack.setPower(0);
            sleep(1000);
        } else {
            telemetry.addData("motorRightBack", "not in configurations");
            telemetry.update();
            sleep(5000);
        }
        if (intakeMotorConfigured) {
            intakeMotor = (DcMotorEx) hardwareMap.dcMotor.get("IntakeMotor");
            intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            intakeMotor.setDirection(DcMotor.Direction.FORWARD);
            intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            intakeMotor.setPower(0.3);
            telemetry.addData("Moving intakeMotor", "forward");
            telemetry.update();
            sleep(1000);
            intakeMotor.setPower(0);
            sleep(1000);
            intakeMotor.setPower(-0.3);
            telemetry.addData("Moving intakeMotor", "backward");
            telemetry.update();
            sleep(1000);
            intakeMotor.setPower(0);
            sleep(1000);
        } else {
            telemetry.addData("intakeMotor", "not in configurations");
            telemetry.update();
            sleep(5000);
        }
        if (transferMotorConfigured) {
            transferMotor = (DcMotorEx) hardwareMap.dcMotor.get("TransferMotor");
            transferMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            transferMotor.setDirection(DcMotorSimple.Direction.FORWARD);
            transferMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            transferMotor.setPower(0.8);
            telemetry.addData("Moving transferMotor", "forward");
            telemetry.update();
            sleep(1000);
            transferMotor.setPower(0);
            sleep(1000);
            transferMotor.setPower(-0.8 );
            telemetry.addData("Moving transferMotor", "backward");
            telemetry.update();
            sleep(1000);
            transferMotor.setPower(0);
            sleep(1000);
        } else {
            telemetry.addData("transferMotor", "not in configurations");
            telemetry.update();
            sleep(5000);
        }
        if (shooterMotorConfigured) {
            shooterMotor = (DcMotorEx) hardwareMap.dcMotor.get("ShooterMotor");
            shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            shooterMotor.setDirection(DcMotor.Direction.FORWARD);
            shooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            shooterMotor.setPower(0.8);
            telemetry.addData("Moving shooterMotor", "forward");
            telemetry.update();
            sleep(700);
            shooterMotor.setPower(0);
            sleep(1000);
            shooterMotor.setPower(0);
            sleep(1000);
        } else {
            telemetry.addData("shooterMotor", "not in configurations");
            telemetry.update();
            sleep(5000);
        }
        if (shooter_ServoConfigured) {
            shooter_Servo = hardwareMap.servo.get("ShooterServo");
            shooter_Servo.setPosition(-0.5);
            shooter_Servo.setPosition(0.5);
            telemetry.addData("Moving", " Shooter Servo");
            telemetry.update();
            sleep(1000);
            shooter_Servo.setPosition(0);
            sleep(2000);
            shooter_Servo.setPosition(-0.5);
            telemetry.update();
            sleep(1000);
            shooter_Servo.setPosition(0);
        } else {
            telemetry.addData("shooterServo", "not in configurations");
            telemetry.update();
            sleep(5000);
        }
        if (wobbleGoalMotorConfigured) {
            wobbleGoalMotor = (DcMotorEx) hardwareMap.dcMotor.get("wobbleGoalMotor");
            wobbleGoalMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            wobbleGoalMotor.setDirection(DcMotorSimple.Direction.FORWARD);
            wobbleGoalMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            wobbleGoalMotor.setPower(0.25);
            telemetry.addData("Moving wobbleGoalMotor", "forward");
            telemetry.update();
            sleep(500);
            wobbleGoalMotor.setPower(0);
            sleep(1000);
            wobbleGoalMotor.setPower(-0.3);
            telemetry.addData("Moving wobbleGoalMotor", "backward");
            telemetry.update();
            sleep(750);
            wobbleGoalMotor.setPower(0);
            sleep(1500);
        } else {
            telemetry.addData("wobbleGoalMotor", "not in configurations");
            telemetry.update();
            sleep(5000);
        }
        if (wobbleGoalServoClawConfigured) {
            wobbleGoalServoClaw = hardwareMap.servo.get("wobbleGoalServoClaw");
            wobbleGoalServoClaw.setPosition(0);
            telemetry.addData("wobbleGoalServoClaw", "open");
            telemetry.update();
            sleep(1000);
            wobbleGoalServoClaw.setPosition(1);
            telemetry.addData("Moving wobbleGoalServoClaw", "closed");
            telemetry.update();
            sleep(1500);
        } else {
            telemetry.addData("wobbleGoalServoClaw", "not in configurations");
            telemetry.update();
            sleep(5000);
        }
        if (webcamServoConfigured) {
            webcamServo = hardwareMap.servo.get("TensorFlowServo");
            webcamServo.setPosition(0.85);
            telemetry.addData("Moving webcamServo", 0.8);
            telemetry.update();
            sleep(1000);
            webcamServo.setPosition(0.65);
            telemetry.addData("Moving webcamServo", 0.65);
            telemetry.update();
            sleep(1000);
            webcamServo.setPosition(0.75);
            telemetry.addData("Moving webcamServo", 0.65);
            telemetry.update();
            sleep(2000);
        } else {
            telemetry.addData("webcamServo", "not in configurations");
            telemetry.update();
            sleep(5000);
        }
        if (rightStickConfigured) {
            rightStick = hardwareMap.servo.get("rightStick");
            rightStick.setPosition(1);
            telemetry.addData("right stick", "down");
            telemetry.update();
            sleep(1000);
            rightStick.setPosition(0);
            telemetry.addData("right stick", "up");
            telemetry.update();
            sleep(1000);
        } else {
            telemetry.addData("rightStick", "not in configurations");
            telemetry.update();
            sleep(5000);
        }
        if (leftStickConfigured) {
            leftStick = hardwareMap.servo.get("leftStick");
            leftStick.setPosition(0);
            telemetry.addData("left stick", "down");
            telemetry.update();
            sleep(1000);
            leftStick.setPosition(1);
            telemetry.addData("left stick", "up");
            telemetry.update();
            sleep(1500);
        } else {
            telemetry.addData("leftStick", "not in configurations");
            telemetry.update();
            sleep(5000);
        }
    }
}