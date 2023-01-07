package org.firstinspires.ftc.teamcode.robots.catbot;

public class Strafe extends Task{
    private Robot robot;
    private double tiles;
    private long oldTime = 0;
    // negative tiles is left
    public Strafe(Robot robot, double tiles)
    {
        this.robot = robot;
        this.tiles = tiles;
    }
    @Override
    public boolean run() {
        if(robot.driveTrain.getMotorFrontLeftPosition() < tiles*STRAFETICKSPERTILE) {
            robot.driveTrain.mechanumDrive(0, (tiles/Math.abs(tiles))*MAXMOTORSPEED, 0);
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
