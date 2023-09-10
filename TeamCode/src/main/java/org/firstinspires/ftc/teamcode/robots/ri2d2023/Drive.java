package org.firstinspires.ftc.teamcode.robots.ri2d2023;

class Drive extends Task {
    private Robot robot;
    private double tiles;
    private double deltaPosition;
    private long oldTime = 0;
    // negative tiles is backwards
    public Drive(Robot robot, double tiles)
    {
        this.robot = robot;
        this.tiles = tiles;
        deltaPosition = robot.driveTrain.getMotorAvgPosition() + Math.abs(tiles*TICKSPERTILE);
    }
    @Override
    public boolean run() {
        if(robot.driveTrain.getMotorAvgPosition() < deltaPosition) {
            robot.driveTrain.mechanumDrive((tiles/Math.abs(tiles))*MAXMOTORSPEED, 0, 0);
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
