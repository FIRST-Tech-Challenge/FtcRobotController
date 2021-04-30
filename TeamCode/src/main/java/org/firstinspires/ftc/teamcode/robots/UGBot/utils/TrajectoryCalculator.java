package org.firstinspires.ftc.teamcode.robots.UGBot.utils;

public class TrajectoryCalculator {
    private double x;
    private double y;
    private double vx, vy;
    private double distance;
    private Constants.Target target;

    public TrajectoryCalculator(double x, double y, Constants.Target target) {
        this.x = x;
        this.y = y;
        this.target = target;
    }

    public void updatePos(double x, double y){
        this.x = x;
        this.y = y;
    }

    public void updateVel(double vx, double vy) {
        this.vx = vx;
        this.vy = vy;
    }

    public void setTarget(Constants.Target target) {
        this.target = target;
    }

    public TrajectorySolution getTrajectorySolution() {
        // initializing base launch height and distance
        double launchHeight = Constants.LAUNCH_HEIGHT;
        TrajectorySolution trajectoryIterationSolution = null;

        // performing iterations for convergence
        for(int i = 0; i < Constants.ITERATIONS; i++) {
            trajectoryIterationSolution = performTrajectoryIteration(launchHeight);

            // calculating new launch height
            launchHeight = Constants.LAUNCH_HEIGHT + getLauncherHeight(trajectoryIterationSolution.getElevation());
        }
        return trajectoryIterationSolution;
    }

    private double getLauncherHeight(double elevation) {
        double c = Math.hypot(Constants.LAUNCHER_VERTICAL_OFFSET, Constants.LAUNCHER_LENGTH);
        return c * Math.sin(Math.toRadians(elevation) + Math.toRadians(Constants.BASE_LAUNCH_ANGLE));
    }

    private TrajectorySolution performTrajectoryIteration(double launchHeight) {
        // vertical distance in meters the disk has to travel
        double travelHeight = target.height - launchHeight;
        // time the disk is in air in seconds
        double flightTime = Math.sqrt((2 * travelHeight) / Constants.GRAVITY);

        // accounting for velocity of robot in x axis
        double xOffset = target.getX() - vx * flightTime;
        // horizontal distance in meters the disk has to travel
        distance = Math.hypot(xOffset - x, target.y - y);
        distance = Math.max(distance, Constants.ILLEGAL_SHOOTING_DISTANCE);

        // using pythagorean theorem to find magnitude of muzzle velocity (in m/s);
        // accounting for velocity of robot in y axis
        double horizontalVelocity = distance / flightTime - vy;
        double verticalVelocity = Constants.GRAVITY * flightTime;
        double velocity = Math.hypot(horizontalVelocity, verticalVelocity);

        // converting tangential velocity in m/s into angular velocity in ticks/s
        double angularVelocity = velocity / Constants.FLYWHEEL_RADIUS; // angular velocity in radians/s
        angularVelocity *= (Constants.ENCODER_TICKS_PER_REVOLUTION / (2 * Math.PI)) * 2; // angular velocity in ticks/s

        double elevation = Math.toDegrees(Math.asin((Constants.GRAVITY * flightTime) / velocity));
        double bearing = Math.toDegrees(Math.atan2((xOffset- x), (target.y -y)));

        return new TrajectorySolution(angularVelocity, elevation, distance, bearing, xOffset, velocity);
    }
}