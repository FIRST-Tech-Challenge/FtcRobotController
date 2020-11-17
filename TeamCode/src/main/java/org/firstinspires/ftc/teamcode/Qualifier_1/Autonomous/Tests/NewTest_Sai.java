/**
 * This test program tests the six chassis motors and the servo.
 *
 * Everyone should run this code when you arrive at the garage to make sure the robot is working properly.
 *
 * Everyone should also run it before leaving to make sure the robot is not broken.
 *
 *
 * @author Sai
 * @version 2.0
 * @since 11/16/2020
 * @status finished
 */

package org.firstinspires.ftc.teamcode.Qualifier_1.Autonomous.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Qualifier_1.Robot;

@Autonomous(name = "NewTest_Sai")
//@Disabled
public class NewTest_Sai extends LinearOpMode {

    @Override
    public void runOpMode() {

        DcMotorEx motorLeftFront;
        DcMotorEx motorRightFront;
        DcMotorEx motorLeftBack;
        DcMotorEx motorRightBack;
        DcMotorEx ShooterMotor;
        DcMotorEx wobbleGoalMotor;
        Servo shooter_Servo;


        motorLeftFront = (DcMotorEx) hardwareMap.dcMotor.get("motorLeftFront");
        motorRightFront = (DcMotorEx) hardwareMap.dcMotor.get("motorRightFront");
        motorLeftBack = (DcMotorEx) hardwareMap.dcMotor.get("motorLeftBack");
        motorRightBack = (DcMotorEx) hardwareMap.dcMotor.get("motorRightBack");
        ShooterMotor = (DcMotorEx) hardwareMap.dcMotor.get("ShooterMotor");
        wobbleGoalMotor = (DcMotorEx) hardwareMap.dcMotor.get("wobbleGoalMotor");
        shooter_Servo = (Servo) hardwareMap.servo.get("ShooterServo");

        // All Motors
        motorLeftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorLeftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ShooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        wobbleGoalMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorLeftFront.setDirection(DcMotor.Direction.FORWARD);
        motorRightFront.setDirection(DcMotor.Direction.FORWARD);
        motorLeftBack.setDirection(DcMotor.Direction.FORWARD);
        motorRightBack.setDirection(DcMotor.Direction.FORWARD);
        ShooterMotor.setDirection(DcMotor.Direction.FORWARD);
        wobbleGoalMotor.setDirection(DcMotor.Direction.FORWARD);

        //Servo
        shooter_Servo.setPosition(0);

        // reset encoder count kept by left motor.
        motorLeftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorLeftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorRightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorRightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ShooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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

        ShooterMotor.setPower(0.5);
        telemetry.addData("Moving Shooter Motor", 0.5);
        telemetry.update();
        sleep(1000);
        ShooterMotor.setPower(0);
        sleep(1000);
        ShooterMotor.setPower(-0.5);
        telemetry.addData("Moving Shooter Motor", -0.5);
        telemetry.update();
        sleep(1000);
        ShooterMotor.setPower(0);

        wobbleGoalMotor.setPower(-0.2);
        telemetry.addData("Moving wobbleGoalMotor Motor", -0.5);
        telemetry.update();
        sleep(300);
        wobbleGoalMotor.setPower(0);
        sleep(1000);
        wobbleGoalMotor.setPower(0.2);
        telemetry.addData("Moving wobbleGoalMotor Motor", 0.5);
        telemetry.update();
        sleep(200);
        wobbleGoalMotor.setPower(0);
        sleep(1000);


        //Tests Servo
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
    }
}