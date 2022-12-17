package org.firstinspires.ftc.teamcode.robots.catbot;

public class Turn implements Task{
    private Robot robot;
    private double degrees;
    // negative tiles is backwards
    public Turn(Robot robot, double degrees)
    {
        this.robot = robot;
        this.degrees = degrees;
    }
    @Override
    public boolean run() {
        if(Math.abs(robot.driveTrain.getMotorFrontLeftPosition()) < Math.abs((degrees/90)*TICKSPER90DEGREES)) {
            robot.driveTrain.mechanumDrive(0, 0, (degrees/Math.abs(degrees)));
            return true;
        }
        return false;
    }
}
