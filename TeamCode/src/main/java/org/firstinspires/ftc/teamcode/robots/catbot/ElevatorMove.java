package org.firstinspires.ftc.teamcode.robots.catbot;

public class ElevatorMove implements Task{
    private Robot robot;
    private char position;
    // position should be top, middle, low, or bottom
    public ElevatorMove(Robot robot, String position)
    {
        this.robot = robot;
        if(position.equals("top"))
            this.position = 'x';
        else if(position.equals("middle"))
            this.position = 'y';
        else if(position.equals("low"))
            this.position = 'b';
        else
            this.position = 'a';
    }
    @Override
    public boolean run() {
        if(position == 'a' && robot.elevatorNClaw.getElevatorPosition() != robot.elevatorNClaw.ELEVTICKSPOS1) {
            robot.elevatorNClaw.elevatorMove(position);
            return true;
        }
        else if(position == 'b' && robot.elevatorNClaw.getElevatorPosition() != robot.elevatorNClaw.ELEVTICKSPOS2) {
            robot.elevatorNClaw.elevatorMove(position);
            return true;
        }
        else if(position == 'y' && robot.elevatorNClaw.getElevatorPosition() != robot.elevatorNClaw.ELEVTICKSPOS3) {
            robot.elevatorNClaw.elevatorMove(position);
            return true;
        }
        else if(position == 'x' && robot.elevatorNClaw.getElevatorPosition() != robot.elevatorNClaw.ELEVTICKSPOS4) {
            robot.elevatorNClaw.elevatorMove(position);
            return true;
        }
        return false;
    }
}
