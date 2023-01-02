package org.firstinspires.ftc.teamcode.robots.catbot;

class Drive implements Task{
    private Robot robot;
    private double tiles;
    // negative tiles is backwards
    public Drive(Robot robot, double tiles)
    {
        this.robot = robot;
        this.tiles = tiles;
    }
    @Override
    public boolean run() {
        if(Math.abs(robot.driveTrain.getMotorFrontLeftPosition()) < Math.abs(tiles*TICKSPERTILE)) {
            robot.driveTrain.mechanumDrive((tiles/Math.abs(tiles))*MAXMOTORSPEED, 0, 0);
            return true;
        }
        return false;
    }
}
