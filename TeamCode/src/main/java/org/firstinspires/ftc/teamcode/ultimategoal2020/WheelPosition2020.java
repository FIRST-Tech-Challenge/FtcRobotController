package org.firstinspires.ftc.teamcode.ultimategoal2020;

import org.firstinspires.ftc.teamcode.ebotsenums.RobotSide;

/**
 * ENUMERATIONS
 */
@Deprecated
public enum WheelPosition2020 {
    //This enumeration captures the wheel configuration information so it can be referenced during
    //construction of a DriveWheel object
    FRONT_LEFT(45, "frontLeft", RobotSide.LEFT),
    FRONT_RIGHT(-45, "frontRight", RobotSide.RIGHT),
    BACK_LEFT(-45, "backLeft", RobotSide.LEFT),
    BACK_RIGHT(45, "backRight", RobotSide.RIGHT);

    private double wheelAngleRadEnum;   //Note:  Wheel angle value stored as radians (multiply by 180/pi for degrees)
    private String motorName;
    private RobotSide robotSide;
    private double spinSign;

    WheelPosition2020(double wheelAngleInDegrees, String motorName, RobotSide robotSide) {
        this.wheelAngleRadEnum = Math.toRadians(wheelAngleInDegrees);
        this.motorName = motorName;
        this.robotSide = robotSide;
        // Positive spin means to turn to the left, meaning that right wheels spin forward and left wheels spin backwards
        this.spinSign = (this.robotSide == RobotSide.RIGHT) ? 1.0 : -1.0;
    }

    public double getWheelAngleRadEnum() {
        return this.wheelAngleRadEnum;
    }

    public String getMotorName() {
        return this.motorName;
    }

    public RobotSide getRobotSide() {
        return this.robotSide;
    }

    public double getSpinSign() {
        return this.spinSign;
    }
}
