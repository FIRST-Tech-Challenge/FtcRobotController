package com.bravenatorsrobotics.core;

import com.qualcomm.robotcore.hardware.DcMotor;

public class RobotSpecifications {

    public final String[] robotMotors;

    public DcMotor.ZeroPowerBehavior zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE;

    public RobotSpecifications(String[] robotMotors) {
        this.robotMotors = robotMotors;
    }

}
