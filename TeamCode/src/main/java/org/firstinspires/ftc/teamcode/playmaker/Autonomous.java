package org.firstinspires.ftc.teamcode.playmaker;

public interface Autonomous {
    Localizer.RobotTransform getStartingTransform();
    ActionSequence getActionSequence();
}
