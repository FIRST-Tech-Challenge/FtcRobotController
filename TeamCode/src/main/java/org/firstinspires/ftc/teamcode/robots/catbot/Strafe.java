package org.firstinspires.ftc.teamcode.robots.catbot;

public class Strafe implements Task{
    private Robot robot;
    private double tiles;
    private static final int TICKSPERTILE = 2800;
    // negative tiles is left
    public Strafe(Robot robot, double tiles)
    {
        this.robot = robot;
        this.tiles = tiles;
    }
    @Override
    public boolean run() {
        if(robot.driveTrain.getMotorFrontLeftPosition() < tiles*TICKSPERTILE) {
            robot.driveTrain.mechanumDrive(0, (tiles/Math.abs(tiles)), 0);
            return true;
        }
        return false;
    }
}
