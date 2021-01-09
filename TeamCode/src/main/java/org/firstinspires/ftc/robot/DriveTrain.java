package org.firstinspires.ftc.robot;

import com.arcrobotics.ftclib.hardware.motors.Motor;

public class DriveTrain {

    private Motor driveLeft, driveRight;

    public DriveTrain(Motor driveLeft, Motor driveRight) {
        this.driveLeft = driveLeft;
        this.driveRight = driveRight;

        driveLeft.setRunMode(Motor.RunMode.VelocityControl);
        driveRight.setRunMode(Motor.RunMode.VelocityControl);
        driveLeft.setVeloCoefficients(0.05, 0, 0);
        driveRight.setVeloCoefficients(0.05, 0, 0);
    }



}
