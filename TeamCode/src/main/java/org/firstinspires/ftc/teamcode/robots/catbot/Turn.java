package org.firstinspires.ftc.teamcode.robots.catbot;

public class Turn implements Task{
    private Robot robot;
    private double degrees;
    private static final int TICKSPERDEGREE = 1000;
    // negative tiles is backwards
    public Turn(Robot robot, double degrees)
    {
        this.robot = robot;
        this.degrees = degrees;
    }
    @Override
    public boolean run() {
        if(robot.driveTrain.getMotorFrontLeftPosition() < (degrees/90)*TICKSPERDEGREE) {
            robot.driveTrain.mechanumDrive(0, 0, (degrees/Math.abs(degrees)));
            return true;
        }
        return false;
    }
}
