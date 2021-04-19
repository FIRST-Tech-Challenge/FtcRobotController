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
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Components.Accesories.WobbleGoal;
import org.firstinspires.ftc.teamcode.Components.BasicChassis;
import org.firstinspires.ftc.teamcode.Robot;

@Autonomous(name = "SoftwareTest ", group="Tests: ")
//@Disabled
public class SoftwareTest extends LinearOpMode {

    @Override
    public void runOpMode() {

        Robot robot = new Robot(this, BasicChassis.ChassisType.ENCODER, false, false);
        Servo webcamServo;

        webcamServo = hardwareMap.servo.get("TensorFlowServo");

        //Servos
        webcamServo.setPosition(0.35);


        ElapsedTime runtime = new ElapsedTime();

        waitForStart();

        //Tests drivetrain motors
        robot.moveForward(1, 0.5);
        telemetry.addData("Moving Robot", "forward");
        telemetry.update();
        sleep(1000);
        robot.moveBackward(1, 0.5);
        telemetry.addData("Moving Robot", "backward");
        telemetry.update();
        sleep(1000);
        robot.moveLeft(1, 0.5);
        telemetry.addData("Moving Robot", "left");
        telemetry.update();
        sleep(1000);
        robot.moveRight(1, 0.5);
        telemetry.addData("Moving RightFront Motor", "backward");
        telemetry.update();
        sleep(1500);

        //Tests intake motor
        //Forward
        robot.startIntake();
        telemetry.addData("intake direction", "forward");
        telemetry.update();
        sleep(1000);
        robot.stopIntake();
        sleep(1000);
        //Backward
        robot.reverseIntake();
        telemetry.addData("intake direction", "backward");
        telemetry.update();
        sleep(1000);
        robot.stopIntake();
        sleep(1500);

        //Tests transfer motor
        //Forward
        robot.startTransfer();
        telemetry.addData("transfer direction", "forward");
        telemetry.update();
        sleep(1000);
        robot.stopTransfer();
        sleep(1000);
        //Backward
        robot.reverseTransfer();
        telemetry.addData("transfer direction", "backward");
        telemetry.update();
        sleep(1000);
        robot.stopTransfer();
        sleep(1500);

        //Tests shooter motor
        robot.shootHighGoal(3);
        telemetry.addData("shooter", "shooting");
        telemetry.update();
        robot.stopShooter();
        sleep(5000);

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
        robot.closeWobbleGoalClaw();
        telemetry.addData("wobbleGoalServoClaw", "closed");
        telemetry.update();
        sleep(1000);
        robot.openWobbleGoalClaw();
        telemetry.addData("wobbleGoalServoClaw", "open");
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
        sleep(1500);

        //Sticks
        robot.moveRightStick(1);
        telemetry.addData("right stick", "down");
        telemetry.update();
        sleep(1000);
        robot.moveRightStick(0);
        telemetry.addData("right stick", "up");
        telemetry.update();
        sleep(1000);
        robot.moveLeftStick(0);
        telemetry.addData("left stick", "down");
        telemetry.update();
        sleep(1000);
        robot.moveLeftStick(1);
        telemetry.addData("left stick", "up");
        telemetry.update();
        sleep(1500);
    }
}