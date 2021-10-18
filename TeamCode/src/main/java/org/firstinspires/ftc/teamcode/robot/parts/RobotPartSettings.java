package org.firstinspires.ftc.teamcode.robot.parts;

import org.firstinspires.ftc.teamcode.robot.Robot;

/**
 * Settings corresponding to a Robot Part
 * @see RobotPart
 * @author 22jmiller
 */
public abstract class RobotPartSettings {
    public boolean useTelemetry;
    public Class className;

    public abstract RobotPart create(Robot robot);
}
