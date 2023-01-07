package org.firstinspires.ftc.teamcode.robots.catbot;

public class ClawMove extends Task{
    private Robot robot;
    private boolean shouldOpen;
    // negative tiles is backwards
    public ClawMove(Robot robot, boolean shouldOpen)
    {
        this.robot = robot;
        this.shouldOpen = shouldOpen;
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
