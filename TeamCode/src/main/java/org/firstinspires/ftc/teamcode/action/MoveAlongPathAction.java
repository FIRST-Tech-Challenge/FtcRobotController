package org.firstinspires.ftc.teamcode.action;

import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.playmaker.Action;
import org.firstinspires.ftc.teamcode.playmaker.Localizer;
import org.firstinspires.ftc.teamcode.playmaker.Localizer.RobotTransform;

import org.firstinspires.ftc.teamcode.playmaker.RobotHardware;

public class MoveAlongPathAction implements Action {

    RobotTransform[] transforms;
    RobotTransform currentTarget;


    public enum FollowPathMethod {
        LINEAR,
    }

    public MoveAlongPathAction(RobotTransform[] transforms, double speed, FollowPathMethod pathMethod) {
        this.transforms = transforms;
    }

    @Override
    public void init(RobotHardware hardware) {
        this.currentTarget = transforms[0];
    }

    @Override
    public boolean doAction(RobotHardware hardware) {

        return currentTarget == null;
    }

    @Override
    public Double progress() {
        return null;
    }

    @Override
    public String progressString() {
        return null;
    }

    @Override
    public Object getActionResult() {
        return null;
    }
}
