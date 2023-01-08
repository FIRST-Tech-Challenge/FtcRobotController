package org.firstinspires.ftc.teamcode.robots.catbot;

public class ClawMove extends Task{
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
            if (robot.elevatorNClaw.getClawPosition() != .5) {
                robot.elevatorNClaw.clawMove(1);
                return true;
            }
            return false;
        }
        else {
            if (robot.elevatorNClaw.getClawPosition() != .2) {
                robot.elevatorNClaw.clawMove(-1);
                return true;
            }
            return false;
        }
    }
}
