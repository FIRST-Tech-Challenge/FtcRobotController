package org.firstinspires.ftc.teamcode.robots.catbot;

public class ElevatorMove extends Task{
    private Robot robot;
    private int position;
    // position should be top, middle, low, or bottom
    public ElevatorMove(Robot robot, String position)
    {
        this.robot = robot;
        if(position.equals("top"))
            this.position = robot.elevatorNClaw.ELEVTICKSPOS4;
        else if(position.equals("middle"))
            this.position = robot.elevatorNClaw.ELEVTICKSPOS3;
        else if(position.equals("low"))
            this.position = robot.elevatorNClaw.ELEVTICKSPOS2;
        else
            this.position = robot.elevatorNClaw.ELEVTICKSPOS1;
    }
    public ElevatorMove(Robot robot, int ticks)
    {
        this.robot = robot;
        this.position = robot.elevatorNClaw.getElevatorPosition()+ticks;
    }
    @Override
    public boolean run() {
        if(robot.elevatorNClaw.getElevatorPosition() != position) {
            robot.elevatorNClaw.elevatorMove(position);
            return true;
        }
        return false;
    }
}
