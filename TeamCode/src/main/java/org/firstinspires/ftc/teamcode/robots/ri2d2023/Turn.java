package org.firstinspires.ftc.teamcode.robots.ri2d2023;

public class Turn extends Task {
    private Robot robot;
    private double degrees;
    private double deltaPosition;
    private long oldTime;
    //negative degrees is left
    public Turn(Robot robot, double degrees)
    {
        this.robot = robot;
        this.degrees = degrees;
        deltaPosition = robot.driveTrain.getMotorAvgPosition() + Math.abs((degrees/90)*TICKSPER90DEGREES);
    }
    @Override
    public boolean run() {
        if(robot.driveTrain.getMotorAvgPosition() < deltaPosition) {
            robot.driveTrain.mechanumDrive(0, 0, (degrees/Math.abs(degrees))*MAXMOTORSPEED);
            return true;
        }
        else {
            if(oldTime == 0)
                oldTime = System.nanoTime();
            if(System.nanoTime() > oldTime+(TIMEBETWEENTASKS*NANOTOSECOND))
                return false;
            return true;
        }
    }
}
