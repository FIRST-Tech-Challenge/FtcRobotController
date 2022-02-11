package org.firstinspires.ftc.teamcode.TeleOp.functions;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

public class driveMethods {
    static DcMotor motorFrontLeft;
    static DcMotor motorBackLeft;
    static DcMotor motorFrontRight;
    static DcMotor motorBackRight;

    public static void setMotors(DcMotor FL, DcMotor BL, DcMotor FR, DcMotor BR){
        motorFrontLeft=FL;
        motorBackLeft = BL;
        motorBackRight = BR;
        motorFrontRight = FR;

    }

    public static void drivetrainAction(Gamepad gamepad1){
        if (Math.abs(gamepad1.left_stick_y) > 0 || Math.abs(gamepad1.left_stick_x) > 0) { //movement

            move(angleOfJoystick(-gamepad1.left_stick_y, gamepad1.left_stick_x),gamepad1);// move method, gets angle from angleOfJoystick

        } else { //turning on spot

            turn(gamepad1);
        }
    }
    public static double angleOfJoystick(double joystickY, double joystickX) { //getting angle of left joystick
        if (joystickY < 0 && joystickX == 0) return 3*3.14159265/2; //back
        if (joystickY >= 0 && joystickX > 0) return Math.atan(Math.abs(joystickY)/ Math.abs(joystickX)); //quadrant 1
        if (joystickY > 0 && joystickX < 0) return Math.atan(Math.abs(joystickY)/ Math.abs(joystickX)) + 3.14159265/2; //quadrant 2
        if (joystickY <= 0 && joystickX < 0) return (Math.atan(Math.abs(joystickY)/ Math.abs(joystickX)) + 3.14159265); //quadrant 3
        if (joystickY < 0 && joystickX > 0) return (Math.atan(Math.abs(joystickY)/ Math.abs(joystickX)) + 3*3.14159265/2); //quadrant 4

        return 3.14159265/2; //forward
    }

    public static void turn(Gamepad gamepad1) { //turning method
        motorFrontLeft.setPower(gamepad1.right_stick_x * 0.5);
        motorBackLeft.setPower(gamepad1.right_stick_x * 0.5);
        motorFrontRight.setPower(-gamepad1.right_stick_x * 0.5);
        motorBackRight.setPower(-gamepad1.right_stick_x * 0.5);
    }

    public static void move(double direction, Gamepad gamepad1) { //move  method
        double amps = 1; //this is the amplitude of the sin function
        double turnMoveMagnitude = 1; // larger values means less turning while moving, can be adjusted
        double speedFactor = 1; //max speed, between 0 and 1

        double hypotenuseLeft = (Math.hypot(-gamepad1.left_stick_y, gamepad1.left_stick_x ) + (gamepad1.right_stick_x/turnMoveMagnitude)) / (1+(Math.ceil(Math.abs(gamepad1.right_stick_x))/turnMoveMagnitude)); //magnitude of left motion
        double hypotenuseRight = (Math.hypot(-gamepad1.left_stick_y, gamepad1.left_stick_x) - (gamepad1.right_stick_x/turnMoveMagnitude)) / (1+(Math.ceil(Math.abs(gamepad1.right_stick_x))/turnMoveMagnitude)); //magnitude of right motion

        if (hypotenuseRight > 1) { //keeps magnitude in bounds just in case
            hypotenuseRight = 1;
        }

        if (hypotenuseLeft > 1) { //keeps magnitude in bounds just in case
            hypotenuseLeft = 1;
        }

        motorFrontLeft.setPower((Math.sin(direction + (3.14159265 / 4)) * (amps * Math.sin(hypotenuseLeft * 2 * Math.PI) + hypotenuseLeft * speedFactor))); //put the b value as 2 pi and add a linear realtionship to the sin function in order to make it work
        motorBackLeft.setPower((Math.sin(direction - (3.14159265 / 4)) * (amps * Math.sin(hypotenuseLeft * 2 * Math.PI) + hypotenuseLeft * speedFactor)));
        motorFrontRight.setPower((Math.sin(direction - (3.14159265 / 4)) * (amps * Math.sin(hypotenuseRight * 2 * Math.PI) + hypotenuseRight * speedFactor)));
        motorBackRight.setPower((Math.sin(direction + (3.14159265 / 4)) * (amps * Math.sin(hypotenuseRight * 2 * Math.PI) + hypotenuseRight * speedFactor)));
        //AN IDEA:
        //I'm thinking that we set some max velocity such as: float MAXvsfrontleft = (Math.sin(direction + (3.14159265 / 4)) * (amps * Math.sin(hypotenuseLeft * 2 * Math.PI) + hypotenuseLeft * speedFactor))
        //and then we have a certain accelartaion such as: double acc = 5;
        //then every time we go through a cycle, we can determine the current velocity from teh accelaration
        //we'll eventually reach the maximum speed through uniform acceleration
        //we can determine the amount of time we need to accelartate for with: t = Maxvsfrontleft/acc;
        //then we simply accelerate for that amount of time
        //we can find the current velocity at each cycle by using uniform accelerated motion equations
        //If we ever move the joystick to another speed, we'll simply set that as a new max and accelerate to that speed.

    }
}
