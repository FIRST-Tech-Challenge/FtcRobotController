package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Constants.sensitivity;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.Constants;

public class MecanumChassis {
    public static DcMotor frontRight;
    public static DcMotor frontLeft;
    public static DcMotor backRight;
    public static DcMotor backLeft;

    public void init(HardwareMap hardwareMap){
        frontRight = hardwareMap.get(DcMotor.class, "fr");
        frontLeft = hardwareMap.get(DcMotor.class, "fl");
        backRight = hardwareMap.get(DcMotor.class, "br");
        backLeft = hardwareMap.get(DcMotor.class, "bl");
    }
    public void drive(Gamepad gamepad){
        //This works, just trust me on it. Slack me or something if you need a full explanation.
        double flPower = (gamepad.left_stick_x + gamepad.left_stick_y + gamepad.right_stick_x);
        double frPower = (-gamepad.left_stick_x + gamepad.left_stick_y - gamepad.right_stick_x);
        double blPower = (-gamepad.left_stick_x + gamepad.left_stick_y + gamepad.right_stick_x);
        double brPower = (gamepad.left_stick_x + gamepad.left_stick_y - gamepad.right_stick_x);

        //This bit seems complicated, but it just gets the maximum absolute value of all the motors.
        double maxPower = Math.max(Math.max(Math.abs(flPower), Math.abs(frPower)), Math.max(Math.abs(blPower), Math.abs(brPower)));

        //If maxPower is less than 1, make it 1. This allows for slower movements.
        maxPower = Math.max(maxPower, 1);

        //Make all of them proportional to the greatest value and factor in the sensitivity.
        flPower = (flPower / maxPower) * sensitivity;
        frPower = (frPower / maxPower) * sensitivity;
        blPower = (blPower / maxPower) * sensitivity;
        brPower = (brPower / maxPower) * sensitivity;

        //Actually set them
        frontLeft.setPower(flPower);
        frontRight.setPower(frPower);
        backLeft.setPower(blPower);
        backRight.setPower(brPower);

    }
}
