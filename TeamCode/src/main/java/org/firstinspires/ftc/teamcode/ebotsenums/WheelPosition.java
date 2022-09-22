package org.firstinspires.ftc.teamcode.ebotsenums;

public enum WheelPosition {

    //This enumeration captures the wheel configuration information so it can be referenced during
    //construction of a DriveWheel object
    FRONT_LEFT(RobotSide.LEFT, "frontLeft"),
    FRONT_RIGHT(RobotSide.RIGHT, "frontRight"),
    BACK_LEFT(RobotSide.LEFT, "backLeft"),
    BACK_RIGHT( RobotSide.RIGHT, "backRight");

    private double wheelAngleRadEnum;   //Note:  Wheel angle value stored as radians (multiply by 180/pi for degrees)
    private String motorName;
    private RobotSide robotSide;
    private double spinSign;

    WheelPosition(RobotSide robotSide, String motorName) {
        this.robotSide = robotSide;
        this.motorName = motorName;
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
