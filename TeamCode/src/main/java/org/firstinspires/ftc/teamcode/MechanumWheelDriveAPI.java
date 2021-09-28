package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;


public class MechanumWheelDriveAPI {
    private DcMotor front_left;
    private DcMotor front_right;
    private DcMotor rear_left;
    private DcMotor rear_right;
    private final float power_scale;

    public MechanumWheelDriveAPI(DcMotor rear_left, DcMotor rear_right, DcMotor front_left, DcMotor front_right) {
        /*
        !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        WARNING: REVERSE YOUR MOTORS! INPUTTING 1 SHOULD
        MAKE THE ROBOT MOVE FORWARD!
        !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        */
        this.rear_left = rear_left;
        this.rear_right = rear_right;
        this.front_left = front_left;
        this.front_right = front_right;
        this.power_scale = 1;
    }

    public double[] convertInputsToPowers(double inputX, double inputY, double inputSpin) {
        // 0 = rear_left, 1 = rear_right, 2 = front_left, 3 = front_right
        double[] output = new double[4];
        double denom = Math.max(Math.abs(inputX) + Math.abs(inputY) + Math.abs(inputSpin), 1);
        output[0] = (inputY - inputX + inputSpin) * this.power_scale / denom;
        output[1] = (inputY + inputX + inputSpin) * this.power_scale / denom;
        output[2] = (inputY + inputX - inputSpin) * this.power_scale / denom;
        output[3] = (inputY - inputX - inputSpin) * this.power_scale / denom;
        return output;
    }

    public void stopAll() {
        rear_left.setPower(0);
        rear_right.setPower(0);
        front_left.setPower(0);
        front_right.setPower(0);
    }

    public double[] runMotorsFromStick(double inputX, double inputY, double inputSpin) {
        double[] output = convertInputsToPowers(inputX, inputY, inputSpin);
        rear_left.setPower(output[0]);
        rear_right.setPower(output[1]);
        front_left.setPower(output[2]);
        front_right.setPower(output[3]);
        return output;
    }
}
