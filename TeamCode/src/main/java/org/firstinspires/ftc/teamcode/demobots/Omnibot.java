package org.firstinspires.ftc.teamcode.demobots;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.RobotLog;

import static java.lang.Math.abs;

/**
 * Demo Omnibot code for showing off the omnibot
 */
@TeleOp(name = "Omnibot", group = "DemoBot")
public class Omnibot extends OpMode {

    //drive train motors
    private DcMotor frontLeft = null;
    private DcMotor backLeft = null;
    private DcMotor frontRight = null;
    private DcMotor backRight = null;

    //bumper speed adjustion for master controls
    private double speed = 2;
    private boolean pressed = false;
    private boolean pressed2 = false;
    private boolean pressedA = false;
    private boolean move = true;

    @Override
    public void init() {
        //defining motors from config
        frontLeft = hardwareMap.dcMotor.get("frontLeft");
        backLeft = hardwareMap.dcMotor.get("backLeft");
        frontRight = hardwareMap.dcMotor.get("frontRight");
        backRight = hardwareMap.dcMotor.get("backRight");

        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        //setting power to the motors to make sure they are not moving
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);

        // Send telemetry message to signify robot is ready to start;
        telemetry.addLine("Robot ready to go");
    }

    @Override
    public void loop() {
        double leftStickX, leftStickY, rightStickX;
        //movement for robot input kid
        double leftStickY1 = gamepad1.left_stick_y;
        double leftStickX1 = gamepad1.left_stick_x;
        //rotation for robot input kid
        double rightStickX1 = gamepad1.right_stick_x;

        //movement for robot input master
        double leftStickY2 = gamepad2.left_stick_y;
        double leftStickX2 = gamepad2.left_stick_x;
        //rotation for robot input master
        double rightStickX2 = gamepad2.right_stick_x;

        //bumper for speeding up and slowing down the robot.
        boolean leftBumper2 = gamepad2.left_bumper;
        boolean rightBumper2 = gamepad2.right_bumper;

        //a input for stopping the robot working for kid TOGGLE
        boolean a2 = gamepad2.a;

        //toggle to stop robot
        if (a2 && !pressedA) {
            move = !move;
            pressedA = true;
        } else if (pressedA && !a2) {
            pressedA = false;
        }

        //slows down robot with master bumper
        if (leftBumper2 && !pressed) {
            speed += 0.2;
            pressed = true;
        }
        // resets pressed for when it isnt pressed
        else if (pressed && !leftBumper2) {
            pressed = false;
        }
        //speeds up robot with master bumper
        if (rightBumper2 && !pressed2) {
            speed -= 0.2;
            pressed2 = true;
        }
        // resets pressed for when it isnt pressed
        else if (pressed2 && !rightBumper2) {
            pressed2 = false;
        }

        //stops divide by 0 error, fastest the robot can go
        if (speed < 1) {
            speed = 1;
        }

        //changing who is driving and move is for stopping kid driving
        if (leftStickX2 == 0 && leftStickY2 == 0 && rightStickX2 == 0 && move) {
            leftStickX = leftStickX1;
            leftStickY = leftStickY1;
            rightStickX = rightStickX1;
        } else {
            leftStickX = leftStickX2;
            leftStickY = leftStickY2;
            rightStickX = rightStickX2;
        }

        //movement for robot method being run
        drive(leftStickY / speed, leftStickX / speed, rightStickX / speed);

        telemetry.addData("Speed", 1 / speed);
        telemetry.addData("override", !move);
        telemetry.update();

        RobotLog.ii("5040MSGHW", "Motors running");
    }


    //method to move the robot
    public void drive(double forward, double sideways, double rotation) {

        //adds all the inputs together to get the number to scale it by
        double scale = abs(rotation) + abs(forward) + abs(sideways);

        //scales the inputs when needed
        if (scale > 1) {
            forward /= scale;
            rotation /= scale;
            sideways /= scale;
        }
        //setting the motor powers to move
        frontLeft.setPower(forward + rotation - sideways);
        backLeft.setPower(forward + rotation + sideways);
        frontRight.setPower(forward - rotation + sideways);
        backRight.setPower(forward - rotation - sideways);
        //Left Front = +Speed + Turn - Strafe      Right Front = +Speed - Turn + Strafe
        //Left Rear  = +Speed + Turn + Strafe      Right Rear  = +Speed - Turn - Strafe
    }
}