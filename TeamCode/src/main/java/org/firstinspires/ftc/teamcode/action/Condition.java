package org.firstinspires.ftc.teamcode.action;

import org.firstinspires.ftc.teamcode.playmaker.RobotHardware;

public interface Condition {
    boolean evaluate(RobotHardware hardware);
}
