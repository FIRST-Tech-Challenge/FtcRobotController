package org.firstinspires.ftc.teamcode.demobots;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;

@TeleOp(name = "Tile Runner", group = "DemoBot")

public class TileRunner extends OpMode {

    private DcMotor frontRight = null;
    private DcMotor backRight = null;
    private DcMotor frontLeft = null;
    private DcMotor backLeft = null;

    //bumper speed adjustion for master controls
    private double speed = 1.5;
    private boolean pressed = false;
    private boolean pressed2 = false;
    private boolean pressedA = false;
    private boolean move = true;

    @Override
    public void init() {
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

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

        double leftStickY, rightStickX;

        //movement for robot input kid
        double leftStickY1 = gamepad1.left_stick_y;
        double rightStickX1 = gamepad1.right_stick_x;

        //movement for robot input master
        double leftStickY2 = gamepad2.left_stick_y;
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
        if (leftStickY2 == 0 && rightStickX2 == 0 && move) {
            leftStickY = leftStickY1;
            rightStickX = rightStickX1;
        } else {
            leftStickY = leftStickY2;
            rightStickX = rightStickX2;
        }

        //movement for robot
        // Left and right motor powers
        double leftPower = leftStickY - rightStickX;
        double rightPower = leftStickY + rightStickX;
        // Setting power to right motors
        frontRight.setPower(Range.clip(rightPower / speed, -1.0, 1.0));
        backRight.setPower(Range.clip(rightPower / speed, -1.0, 1.0));
        // Setting power to left motors
        frontLeft.setPower(Range.clip(leftPower / speed, -1.0, 1.0));
        backLeft.setPower(Range.clip(leftPower / speed, -1.0, 1.0));

        telemetry.addData("Speed", 1 / speed);
        telemetry.addData("override", !move);
        telemetry.update();

        RobotLog.ii("5040MSGHW", "Motors running");
    }
}

