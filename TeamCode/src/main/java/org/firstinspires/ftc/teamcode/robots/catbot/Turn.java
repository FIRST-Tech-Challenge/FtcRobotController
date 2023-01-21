package org.firstinspires.ftc.teamcode.robots.catbot;

public class Turn extends Task{
    private Robot robot;
    private double degrees;
    private long oldTime;
    //negative degrees is left
    public Turn(Robot robot, double degrees)
    {
        this.robot = robot;
        this.degrees = degrees;
    }
    @Override
    public boolean run() {
        if(robot.driveTrain.getMotorAvgPosition() < Math.abs((degrees/90)*TICKSPER90DEGREES)) {
            robot.driveTrain.mechanumDrive(0, 0, (degrees/Math.abs(degrees))*MAXMOTORSPEED);
            return true;
        }
        else {
            if(oldTime == 0)
                oldTime = System.nanoTime();
            if(System.nanoTime() > oldTime+(.25*NANOTOSECOND))
                return false;
            return true;
        }
    }
}
