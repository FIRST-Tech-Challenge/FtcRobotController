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

import org.firstinspires.ftc.teamcode.Components.Accesories.WobbleGoal;
import org.firstinspires.ftc.teamcode.Components.BasicChassis;
import org.firstinspires.ftc.teamcode.Robot;

@Autonomous(name = "WalrusTest ", group="Tests: ")
//@Disabled
public class WalrusTest extends LinearOpMode {

    @Override
    public void runOpMode() {

        Robot robot = new Robot(this, BasicChassis.ChassisType.ENCODER, false, false);
        DcMotorEx motorLeftFront;
        DcMotorEx motorRightFront;
        DcMotorEx motorLeftBack;
        DcMotorEx motorRightBack;
        DcMotorEx intakeMotor;
        Servo webcamServo;


        motorLeftFront = (DcMotorEx) hardwareMap.dcMotor.get("motorLeftFront");
        motorRightFront = (DcMotorEx) hardwareMap.dcMotor.get("motorRightFront");
        motorLeftBack = (DcMotorEx) hardwareMap.dcMotor.get("motorLeftBack");
        motorRightBack = (DcMotorEx) hardwareMap.dcMotor.get("motorRightBack");
        webcamServo = (Servo) hardwareMap.servo.get("TensorFlowServo");


        // Drivetrain Motors
        motorLeftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorLeftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Servos
        webcamServo.setPosition(0.35);

        // To match default
        motorLeftFront.setDirection(DcMotor.Direction.REVERSE);
        motorRightFront.setDirection(DcMotor.Direction.FORWARD);
        motorLeftBack.setDirection(DcMotor.Direction.REVERSE);
        motorRightBack.setDirection(DcMotor.Direction.FORWARD);


        // make the drivetrains motors run on only power
        motorLeftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorLeftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorRightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorRightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

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
        robot.startIntake();
        sleep(1000);
        robot.stopIntake();
        sleep(1000);
        //Backwards
        robot.reverseIntake();
        sleep(1000);
        robot.stopIntake();
        sleep(1500);

        //Tests transfer motor
        //Forwards
        robot.startTransfer();
        sleep(1000);
        robot.stopTransfer();
        sleep(1000);
        //Backwards
        robot.reverseTransfer();
        sleep(1000);
        robot.stopTransfer();
        sleep(1500);

        //Tests shooter motor
        robot.shootHighGoal(3);
        robot.stopShooter();

        //Moving wobbleGoalMotor
        robot.moveWobbleGoalToPosition(WobbleGoal.Position.GRAB);
        telemetry.addData("wobbleGoalMotor", "grab position");
        telemetry.update();
        sleep(1000);
        robot.moveWobbleGoalToPosition(WobbleGoal.Position.RAISE);
        telemetry.addData("wobbleGoalMotor", "raise position");
        telemetry.update();
        sleep(1000);
        robot.moveWobbleGoalToPosition(WobbleGoal.Position.REST);
        telemetry.addData("wobbleGoalMotor", "rest position");
        telemetry.update();
        sleep(1500);

        //Wobble Goal Servo Claw
        robot.openWobbleGoalClaw();
        telemetry.addData("wobbleGoalServoClaw", "open");
        telemetry.update();
        sleep(1000);
        robot.closeWobbleGoalClaw();
        telemetry.addData("wobbleGoalServoClaw", "closed");
        telemetry.update();
        sleep(1500);

        //Webcam Servo
        webcamServo.setPosition(0.5);
        telemetry.addData("Moving webcamServo", 0.5);
        telemetry.update();
        sleep(1000);
        webcamServo.setPosition(0.35);
        telemetry.addData("Moving webcamServo", 0.35);
        telemetry.update();
        sleep(2000);

        //Sticks
        robot.moveRightStick(0);
        robot.moveRightStick(1);
        robot.moveLeftStick(0);
        robot.moveLeftStick(1);
    }
}