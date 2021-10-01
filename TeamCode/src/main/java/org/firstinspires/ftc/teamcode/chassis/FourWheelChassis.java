package org.firstinspires.ftc.teamcode.chassis;

import static org.firstinspires.ftc.teamcode.Constants.sensitivity;

import com.qualcomm.robotcore.hardware.Gamepad;

public class FourWheelChassis extends Chassis {
    @Override
    public void drive(Gamepad gamepad){
        //This works, just trust me on it. Slack me or something if you need a full explanation.
        double lPower = (gamepad.left_stick_y + gamepad.right_stick_x);
        double rPower = (gamepad.left_stick_y - gamepad.right_stick_x);

        //This bit seems complicated, but it just gets the maximum absolute value of all the motors.
        double maxPower = Math.max(Math.abs(lPower), Math.abs(rPower));

        //If maxPower is less than 1, make it 1. This allows for slower movements.
        maxPower = Math.max(maxPower, 1);

        //Make all of them proportional to the greatest value and factor in the sensitivity.
        lPower = (lPower / maxPower) * sensitivity;
        rPower = (rPower / maxPower) * sensitivity;

        //Actually set them
        frontLeft.setPower(lPower);
        frontRight.setPower(rPower);
        backLeft.setPower(lPower);
        backRight.setPower(rPower);

    }
}
