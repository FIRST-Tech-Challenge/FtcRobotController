package org.firstinspires.ftc.teamcode.robots.catbot;

class Drive implements Task{
    private Robot robot;
    private double tiles;
    private static final int TICKSPERTILE = 2500;
    // negative tiles is backwards
    public Drive(Robot robot, double tiles)
    {
        this.robot = robot;
        this.tiles = tiles;
    }
    @Override
    public boolean run() {
        if(robot.driveTrain.getMotorFrontLeftPosition() < tiles*TICKSPERTILE) {
            robot.driveTrain.mechanumDrive((tiles/Math.abs(tiles)), 0, 0);
            return true;
        }
        return false;
    }
}
