package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public abstract class DriveMethods extends OpMode {
    Devices robot = new Devices();

    /**
     * Given the desired movement, sets the power to each wheel to match that as well as it can.
     * @param axial describes the backward and forward movement
     * @param lateral describes left and right movement (strafe)
     * @param yaw describes a spinning motion
     */
    public void omniDrive(double axial, double lateral, double yaw) {
        // code copied from BasicOmniOpMode_Linear.java
        double max;

        double leftFrontPower  = axial + lateral + yaw;
        double rightFrontPower = axial - lateral - yaw;
        double leftBackPower   = axial - lateral + yaw;
        double rightBackPower  = axial + lateral - yaw;

        max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower  /= max;
            rightFrontPower /= max;
            leftBackPower   /= max;
            rightBackPower  /= max;
        }

        robot.leftFrontDrive.setPower(leftFrontPower);
        robot.rightFrontDrive.setPower(rightFrontPower);
        robot.leftBackDrive.setPower(leftBackPower);
        robot.rightBackDrive.setPower(rightBackPower);
    }
    public void omniOp(double wormrote) {
        robot.motorTest.setPower(wormrote);
    }
    //wormrote variable controls vertical movement of the worm gear.
}

//Yaw=turn, Lateral=SideToSide,  Axial is Forward
