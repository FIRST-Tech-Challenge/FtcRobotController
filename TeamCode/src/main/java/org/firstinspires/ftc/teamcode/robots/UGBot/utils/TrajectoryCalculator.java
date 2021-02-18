package org.firstinspires.ftc.teamcode.robots.UGBot.utils;

public class TrajectoryCalculator {
    private double x;
    private double y;
    private double distance;

    public TrajectoryCalculator(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public void updatePos(double x, double y){
        this.x = x;
        this.y = y;
    }

    public TrajectorySolution getTrajectorySolution() {

        distance = Math.sqrt(Math.pow((Constants.goalX-x),2) + Math.pow((Constants.goalY-y),2));

        // vertical distance in meters the disk has to travel
        double travelHeight = Constants.GOAL_HEIGHT - Constants.LAUNCH_HEIGHT;
        // time the disk is in air in seconds
        double flightTime = Math.sqrt((2 * travelHeight) / Constants.GRAVITY);

        // using pythagorean theorem to find magnitude of muzzle velocity (in m/s);
        double horizontalVelocity = distance / flightTime;
        double verticalVelocity = Constants.GRAVITY * flightTime;
        double velocity = Math.sqrt(Math.pow(horizontalVelocity, 2) + Math.pow(verticalVelocity, 2));

        // converting tangential velocity in m/s into angular velocity in ticks/s
        double angularVelocity = velocity / Constants.FLYWHEEL_RADIUS; // angular velocity in radians/s
        angularVelocity *= (Constants.ENCODER_TICKS_PER_REVOLUTION / (2 * Math.PI)) * 2; // angular velocity in ticks/s

        double elevation = Math.toDegrees(Math.asin((Constants.GRAVITY * flightTime) / velocity));
        double bearing = Math.toDegrees(Math.atan2((Constants.goalX-x), (Constants.goalY-y)));
        return new TrajectorySolution(angularVelocity, elevation, distance, bearing);
    }
}