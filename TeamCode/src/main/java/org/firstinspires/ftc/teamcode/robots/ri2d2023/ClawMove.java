package org.firstinspires.ftc.teamcode.robots.ri2d2023;

public class ClawMove extends Task {
    private Robot robot;
    private boolean shouldOpen;
    // negative tiles is backwards
    public ClawMove(Robot robot, boolean open)
    {
        this.robot = robot;
        this.shouldOpen = open;
    }
    @Override
    public boolean run() {
        if(shouldOpen) {
            if (robot.intake.getClawPosition() != .5) {
                robot.intake.clawOpen();
                return true;
            }
            return false;
        }
        else {
            if (robot.intake.getClawPosition() != .2) {
                robot.intake.clawClose();
                return true;
            }
            return false;
        }
    }
}
