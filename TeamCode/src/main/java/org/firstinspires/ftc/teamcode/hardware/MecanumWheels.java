package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.Gamepad;

/**
 * THis is the class to caculate the power sending to the wheel motor. .
 * This fits for the wheel configuration of :
 *   The wheels need to form an "X" configuration
 */
public class MecanumWheels {

    //Initialize wheel motor power to 0
    public double wheelFrontRightPower = 0;
    public double wheelFrontLeftPower = 0;
    public double wheelBackRightPower = 0;
    public double wheelBackLeftPower = 0;

    public double frontWheelWeaken = 1;

    public double turbo = 0;

    public MecanumWheels(){ }

    /**
     * This method calculate the wheel according to the input of game pad.
     *
     * @param gamepad1  This is the game pad object, that contains input of game pad
     * @param turbo  Power factor.  Range is 0-1
     */
    public void joystick(Gamepad gamepad1, double turbo){

        this.turbo = turbo;

        double x1 = gamepad1.left_stick_x; //Math.pow(gamepad1.left_stick_x,3.0);
        double y1 = gamepad1.left_stick_y; //Math.pow(gamepad1.left_stick_y,3.0);
        double x2 = gamepad1.right_stick_x; //Math.pow(gamepad1.right_stick_x,3.0);
        wheelFrontRightPower  = -y1 - x2 - x1;
        wheelBackRightPower   = y1 + x2 - x1;
        wheelFrontLeftPower   = -y1 + x2 + x1;
        wheelBackLeftPower    = y1 - x2 + x1;

        double max = Math.max(Math.abs(wheelFrontRightPower), Math.max(Math.abs(wheelBackRightPower),
                Math.max(Math.abs(wheelFrontLeftPower), Math.abs(wheelBackLeftPower))));

        if (max > 1.0)
        {
            wheelFrontRightPower /= max;
            wheelBackRightPower  /= max;
            wheelFrontLeftPower  /= max;
            wheelBackLeftPower   /= max;
        }

        wheelFrontRightPower *= turbo;
        wheelBackRightPower  *= turbo;
        wheelFrontLeftPower  *= turbo;
        wheelBackLeftPower   *= turbo;
    }

    public void forward(double turbo){
        this.turbo = turbo;
        wheelFrontRightPower = 1 * turbo;
        wheelFrontLeftPower = 1 * turbo;
        wheelBackRightPower = 1 * turbo;
        wheelBackLeftPower = 1 * turbo;
    }

    public void backwards(double turbo){
        this.turbo = turbo;
        wheelFrontRightPower = -1 * turbo;
        wheelFrontLeftPower = -1 * turbo;
        wheelBackRightPower = -1 * turbo;
        wheelBackLeftPower = -1 * turbo;
    }

    public void strafeRight(double turbo){
        this.turbo = turbo;
        wheelFrontRightPower = -1 * turbo * frontWheelWeaken;
        wheelFrontLeftPower = 1 * turbo * frontWheelWeaken;
        wheelBackRightPower = 1 * turbo;
        wheelBackLeftPower = -1 * turbo;
    }

    public void strafeLeft(double speed){
        this.turbo = turbo;
        wheelFrontRightPower = 1 * turbo * frontWheelWeaken;
        wheelFrontLeftPower = -1 * turbo * frontWheelWeaken;
        wheelBackRightPower = -1 * turbo;
        wheelBackLeftPower = 1 * turbo;
    }

    public void turnRight(double speed){
        this.turbo = turbo;
        wheelFrontRightPower = .6 / speed;
        wheelFrontLeftPower = .6 / speed;
        wheelBackRightPower = -.6 / speed;
        wheelBackLeftPower = -.6 / speed;
    }

    public void turnLeft(double speed){
        this.turbo = turbo;
        wheelFrontRightPower = -.6 / speed;
        wheelFrontLeftPower = -.6 / speed;
        wheelBackRightPower = .6 / speed;
        wheelBackLeftPower = .6 / speed;
    }

    public void tiltRight(double speed){
        this.turbo = turbo;
        wheelFrontLeftPower = 1 / speed;
        wheelFrontRightPower = 0;
        wheelBackRightPower = 1 / speed;
        wheelBackLeftPower = 0;
    }

    public void tiltLeft(double speed){
        wheelBackLeftPower = 1 / speed;
        wheelBackRightPower = 0;
        wheelFrontRightPower = 1 / speed;
        wheelBackLeftPower = 0;
    }

    public void stop(){
        wheelBackLeftPower = 0;
        wheelBackRightPower = 0;
        wheelFrontRightPower = 0;
        wheelFrontLeftPower = 0;
    }
}