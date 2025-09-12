package org.firstinspires.ftc.teamcode.Autonomous;

import static org.firstinspires.ftc.teamcode.riptideUtil.LAT_KD;
import static org.firstinspires.ftc.teamcode.riptideUtil.LAT_KI;
import static org.firstinspires.ftc.teamcode.riptideUtil.LAT_KP;
import static org.firstinspires.ftc.teamcode.riptideUtil.MAX_A;
import static org.firstinspires.ftc.teamcode.riptideUtil.MAX_V;
import static org.firstinspires.ftc.teamcode.riptideUtil.TURN_KD;
import static org.firstinspires.ftc.teamcode.riptideUtil.TURN_KI;
import static org.firstinspires.ftc.teamcode.riptideUtil.TURN_KP;
import static org.firstinspires.ftc.teamcode.riptideUtil.VERT_KD;
import static org.firstinspires.ftc.teamcode.riptideUtil.VERT_KI;
import static org.firstinspires.ftc.teamcode.riptideUtil.VERT_KP;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Autonomous.Utils.Path;
import org.firstinspires.ftc.teamcode.Autonomous.Utils.TrapezoidalMotionProfile;
import org.firstinspires.ftc.teamcode.Autonomous.Utils.Waypoint;
import org.firstinspires.ftc.teamcode.Modules.PIDController;
import org.firstinspires.ftc.teamcode.Modules.Utils.EditablePose2D;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.riptideUtil;
import org.firstinspires.ftc.teamcode.Modules.Drivetrain;

// maybe GOING TO BE REFACTORED, THIS IS GOING TO BE SO DEPRECATED

public class AutonomousRobot extends Robot {
    Drivetrain drivetrain;
    Robot robot;
    public AutonomousRobot(HardwareMap hardwareMap) {
        super(hardwareMap);
    }

    /**
     * @param curr, robot current heading in degrees
     * @param goal, robot goal heading, global, in degrees
     * @return an angle within 180 and -180
     */
    private double normalizeAngleError(double curr, double goal) {
        double error = goal - curr;
        error = ((error + 180) % 360 + 360) % 360 - 180;
        return error;
    }

    /**
     * @param robotPos Pose2D of the robot
     * @param point    Pose2D of our goal point (note that we don't need the heading).
     * @return True if the robot is within the determined boundaries of a point
     */
    public boolean atPoint(EditablePose2D robotPos, EditablePose2D point) {
        double pointTolerance = riptideUtil.POINT_TOLERANCE;

        double dx = point.getX(DistanceUnit.INCH) - robotPos.getX(DistanceUnit.INCH);
        double dy = point.getY(DistanceUnit.INCH) - robotPos.getY(DistanceUnit.INCH);

        double distance = Math.sqrt(dx * dx + dy * dy);
        return distance <= pointTolerance;
    }



    public enum moveStates {
        GOTOPOINT,
        STAYATPOINT,
        PIDPOINTMETHOD,
        PUREPURSUIT,
        CALCULATE,
        IDLE,
        ONLYACTION
    }

    public moveStates currentState = moveStates.IDLE;

    PIDController latPid = new PIDController(riptideUtil.LAT_KP, riptideUtil.LAT_KI, riptideUtil.LAT_KD);
    PIDController vertPid = new PIDController(riptideUtil.VERT_KP, riptideUtil.VERT_KI, riptideUtil.VERT_KD);
    PIDController turnPid = new PIDController(TURN_KP, TURN_KI, TURN_KD);
    private ElapsedTime timer;
    private double goalPosX;
    private double goalPosY;

    private double posKP = 0;
    private double posKI = 0;
    private double posKD = 0;

    PIDController positionPID = new PIDController(posKP, posKI, posKD);
    TrapezoidalMotionProfile trapezoidalMotionProfile = new TrapezoidalMotionProfile(0, 0);



    public Path path = new Path.PathBuilder()
            .addNewFullPoint(
                    new Waypoint(0, 0, 90, 36, 72, DistanceUnit.INCH),
                    () -> {
                        // some function here
                    },
                    0
            )
            .build();

    TrapezoidalMotionProfile motionProfile = new TrapezoidalMotionProfile(MAX_A, MAX_V);

    private double getAngleError(double targetAngle, double currentAngle) {
        double error = targetAngle - currentAngle;
        error = ((error + 180) % 360 + 360) % 360 - 180;
        return error;
    }

    private EditablePose2D s;

    private int pathIndex = 0;
    private double lineSlope = 0;
    private double time = System.nanoTime() / (Math.pow(10, 9));
    double elapsedTime = 0;

    EditablePose2D start = new EditablePose2D(0, 0, 0, DistanceUnit.INCH);
    Path.PathPoint goalPoint;

    //Main AutonomousFSM
    public void step() {

        switch (currentState) {
            case ONLYACTION:
                path.get(pathIndex).getAction().Action();
                currentState = moveStates.PIDPOINTMETHOD;
                break;
            case IDLE:
                break;
            case STAYATPOINT:
                break;
            case GOTOPOINT:


                //Find the current field positions
                double currentX = this.robot.getDrivetrain().getCurrPos().getX(DistanceUnit.INCH);
                double currentY = this.robot.getDrivetrain().getCurrPos().getY(DistanceUnit.INCH);
                double currentH = this.robot.getDrivetrain().getCurrPos().getH();

                //Find the expected position along the path, as a magnitude of a vector with angle lineSlope
                // then use some trig to find x and y components

                elapsedTime = System.nanoTime() / 1e9 - time;

                double magnitude = motionProfile.getExpectedPosition(elapsedTime);
                double xComponent = start.getX(DistanceUnit.INCH) + magnitude * Math.cos(lineSlope);
                double yComponent = start.getY(DistanceUnit.INCH) + magnitude * Math.sin(lineSlope);

                //Set Pids to what they are supposed to be.a
                vertPid.setPID(VERT_KP, VERT_KI, VERT_KD);
                latPid.setPID(LAT_KP, LAT_KI, LAT_KD);
                turnPid.setPID(TURN_KP, TURN_KI, TURN_KD);

                //shift the errors to robot centric errors
                double xError = xComponent - currentX;
                double yError = yComponent - currentY;

                double xErrorRot = xError * Math.cos(-currentH) - yError * Math.sin(-currentH); // I could probably change this into -sin
                double yErrorRot = xError * Math.sin(-currentH) + yError * Math.cos(-currentH); // bc sin is odd but it would be too complicated
                // to understand again if took a quick look
                //Find the forward and strafe powers
                double vertPower = vertPid.calculate(0, xErrorRot);
                double latPower = latPid.calculate(0, yErrorRot);

                //Find anglePower
                double angleError = getAngleError(Math.toDegrees(goalPoint.getWaypoint().getH()), Math.toDegrees(currentH));
                double anglePower = turnPid.calculate(0, angleError);

                if(goalPoint.getAction() != null){
                    goalPoint.getAction().Action();
                }

                setWheelPowers(vertPower, latPower, anglePower);

                currentState = moveStates.PIDPOINTMETHOD;

                break;
            case CALCULATE: //Calculates the profile from the current position to path index

                if(goalPoint.getWaypoint() == null){
                    currentState = moveStates.ONLYACTION;
                    break;
                }

                //set current position point as start point
                start = new EditablePose2D(
                        this.robot.getDrivetrain().getCurrPos().getX(DistanceUnit.INCH),
                        this.robot.getDrivetrain().getCurrPos().getY(DistanceUnit.INCH),
                        this.robot.getDrivetrain().getCurrPos().getH(),
                        DistanceUnit.INCH
                );

                //Find distance between two points for motion profile
                double dx = goalPoint.getWaypoint().getX(DistanceUnit.INCH) - start.getX(DistanceUnit.INCH);
                double dy = goalPoint.getWaypoint().getY(DistanceUnit.INCH) - start.getY(DistanceUnit.INCH);

                //Find the angle from the x axis for global coordinates - Used to find expected position in the future
                double distance = Math.sqrt(dy * dy + dx * dx);

                //Find the angle from the x axis for global coordinates - Used to find expected position in the future
                lineSlope = Math.atan2(dy, dx);

                //Set the motion profile with the new calculated distance
                motionProfile.setProfile(goalPoint.getWaypoint().getMaxAccel(), goalPoint.getWaypoint().getGoalVelocity());
                motionProfile.calculateProfile(distance);

                //Clear pid integral windups
                latPid.resetIntegral();
                vertPid.resetIntegral();
                turnPid.resetIntegral();

                // reset the elapsed time for this path.
                time = System.nanoTime() / (Math.pow(10, 9));

                currentState = moveStates.PIDPOINTMETHOD;

                break;
            case PUREPURSUIT:
                //input velocity
                //scale vectors
                break;
            case PIDPOINTMETHOD:

                goalPoint = path.get(pathIndex);
                elapsedTime = (System.nanoTime() / (Math.pow(10, 9)) - time);

                cP = super.getDrivetrain().getCurrPos();
                boolean y = atPoint(super.getDrivetrain().getCurrPos(), goalPoint.getWaypoint());

                atPoint = y;
                delayUntilNextPointClear =  elapsedTime > goalPoint.getDelayUntilNextPoint();
                notAtEndOfPath = pathIndex < path.getPathSize() - 1;


                if (y && elapsedTime > goalPoint.getDelayUntilNextPoint()) {
                    if(pathIndex < path.getPathSize() - 1){
                        pathIndex = pathIndex + 1;
                        goalPoint = path.get(pathIndex);
                        time = System.nanoTime() / Math.pow(10, 9);
                        elapsedTime = 0;
                        currentState = moveStates.CALCULATE;
                        break;
                    }
                }


                currentState = moveStates.GOTOPOINT;
                break;
        }
    }

    boolean atPoint = true;
    boolean delayUntilNextPointClear = false;
    boolean notAtEndOfPath = false;
    EditablePose2D cP = new EditablePose2D(0,0,0,DistanceUnit.INCH);

    //Debug methods
    public int pathsize(){
        return path.getPathSize();
    }

    public boolean isAtPoint(){
        return atPoint;
    }

    public boolean pastDelayUntilNextPoint(){
        return delayUntilNextPointClear;
    }

    public boolean isNotAtEndOfPath(){
        return notAtEndOfPath;
    }

    public double getElapsedTime(){
        return elapsedTime;
    }

    public int getPathIndex(){
        return pathIndex;
    }

    public Waypoint autonPos(){
        return goalPoint.getWaypoint();
    }

    public EditablePose2D autonrealPos(){
        return cP;
    }

    public double getDelay(){
        return goalPoint.getDelayUntilNextPoint();
    }

    public double getTime(){
        return time;
    }

    public void setWheelPowers(double vertPower, double latPower, double anglePower) {
        double denominator = Math.max(Math.abs(latPower) + Math.abs(vertPower) + Math.abs(anglePower), 1);
        double frWheelPower = (vertPower + latPower + anglePower) / denominator;
        double flWheelPower = (vertPower - latPower - anglePower) / denominator;
        double brWheelPower = (vertPower - latPower + anglePower) / denominator;
        double blWheelPower = (vertPower + latPower - anglePower) / denominator;

        super.getDrivetrain().setWheelPowers(flWheelPower, frWheelPower, brWheelPower, blWheelPower);
    }

    public void startPath() {
        pathIndex = 0;
        goalPoint = path.get(pathIndex);
        currentState = moveStates.CALCULATE;

        Path.PathPoint firstPoint = path.get(0);
        Waypoint firstPose = firstPoint.getWaypoint();

        if (atPoint(firstPose, robot.getDrivetrain().getCurrPos())) {
            pathIndex = 1;
        }
    }

    public Path getPath() {
        return path;
    }

    public void setPath(Path path) {
        this.path = path;
    }

    public void resetTimer() {
        timer.reset();
        positionPID.reset();
    }

    public void calculateDistance(double distance) {
        trapezoidalMotionProfile.calculateProfile(distance);
    }


}
