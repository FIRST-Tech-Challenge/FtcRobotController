package org.firstinspires.ftc.teamcode.Testing.Helper_test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class moveRobotToApril extends LinearOpMode {
    motorCmds MoveCmd = new motorCmds();

    //---------------------------------------------
    // Move robot according to desired axes motions
    // Positive X is forward
    // Positive Y is strafe left
    // Positive Yaw is counter-clockwise
    //---------------------------------------------
    public void moveRobotToApril(double x, double y, double yaw) {
        // Calculate wheel powers.
        double lfPower  =  x + y + yaw;
        double rfPower  =  x - y - yaw;
        double lbPower  =  x - y + yaw;
        double rbPower  =  x + y - yaw;

        // Normalize wheel powers to be less than 1.0
        double max = Math.max(Math.abs(lfPower), Math.abs(rfPower));
        max = Math.max(max, Math.abs(lbPower));
        max = Math.max(max, Math.abs(rbPower));

        if (max > 1.0) {
            lfPower /= max;
            rfPower /= max;
            lbPower /= max;
            rbPower /= max;
        }

        // Send powers to the wheels.
        MoveCmd.lfDrive.setPower(lfPower);
        MoveCmd.rfDrive.setPower(rfPower);
        MoveCmd.lbDrive.setPower(lbPower);
        MoveCmd.rbDrive.setPower(rbPower);
    }
    @Override
    public void runOpMode() {
    }
}
