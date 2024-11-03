package org.firstinspires.ftc.teamcode.opmodes.autonomous.base;

import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.trajectory.Trajectory;
import com.arcrobotics.ftclib.trajectory.TrajectoryConfig;
import com.arcrobotics.ftclib.trajectory.TrajectoryGenerator;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.command.SounderBotBaseRunCommand;
import org.firstinspires.ftc.teamcode.opmodes.OpModeTemplate;
import org.firstinspires.ftc.teamcode.opmodes.autonomous.Tunables;
import org.firstinspires.ftc.teamcode.opmodes.autonomous.driver.AutoRobotDriver;
import org.firstinspires.ftc.teamcode.opmodes.autonomous.driver.RobotCentricDriver;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.AutoFourWheelMecanumDriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.feedback.DriverFeedback;

import java.util.List;
import java.util.Locale;
import java.util.Stack;
import android.util.Log;
/**
 * <p>This is main class for auto op mode. It register the callback for each control hub scan cycle
 * (typically 50ms per scan), for each cycle, it will follow trajectory or perform action based on current state</p>
 *
 *  <p>It have 2 abstract methods: <b>{@link #setupWaypointsAndActions()}</b> & <b>{@link #updateTargetPose(Pose2d, Pose2d)}</b></p>
 *  <li><b>{@link #setupWaypointsAndActions()}</b>: will be invoked once at initialize time to setup stack of trajectory & actions</li>
 *  <li><b>{@link #updateTargetPose(Pose2d, Pose2d)}</b>: will be invoked in each cycle to update targetX, targetY, and targetRotationSpeed. Different type
 *  of control (PID or simple) should override this method to provide different algorithm to calculate target position
 *  </li>
 */
public abstract class AutoOpMode extends OpModeTemplate {

    private static final String LOG_TAG = AutoOpMode.class.getName();
    private final String TELEMETRY_CAPTION_PREFIX = "[v4-" + getClass().getSimpleName() + "] ";
    private final ElapsedTime timer = new ElapsedTime();
    protected AutoFourWheelMecanumDriveTrain driveTrain;

    protected DriverFeedback driverFeedback;

    protected TrajectoryConfig config;
    protected Stack<List<Pose2d>> wayPointsStack = new Stack<>();
    protected Stack<RunCommand> actions = new Stack<>();
    protected double targetX, targetY, targetRotationSpeed;
    private Trajectory currentTrajectory;
    private RunCommand currentAction;
    private AutoRobotDriver robotDriver;
    private State currentState;
    private boolean finished = false;
    private int trajectoryCount = 0;

    /**
     * convert angle to turn to turning speed. No matter it is robotCentric or fieldCentric
     * the rotation parameter is in form of rotation speed not angle, but the difference from
     * odometer are the angle, so use this method to convert to speed.<p/>
     * The reason to use multiplier is: <br/>
     * we know the formula:
     * <p>
     * <b>rotation_speed = rotation_angle / t = rotation * (1/t)</b> </p>
     * Because scan interval is a constant value (could be different depends on model, but for a
     * certain hardware, the interval is constant), meaning the t is a constant value,
     * then 1/t is a constant value, we could define
     * <p><b>multiplier = (1/t)</b></p>
     * then the equation becomes:
     * <p><b>rotation_speed = rotation_angle * multiplier</b></p>
     *
     * Multiplier should be tuned to match the hardware
     */
    protected static double rotateAngleToSpeed(double angle) {
        double speed = angle * Tunables.ANGLE_TO_SPEED_MULTIPLIER;
        // make sure the speed is between -MAX_ROTATION_SPEED and MAX_ROTATION_SPEED
        return Math.max(-Tunables.MAX_ROTATION_SPEED, Math.min(speed, Tunables.MAX_ROTATION_SPEED));
    }

    @Override
    public void initialize() {
        driverFeedback = new DriverFeedback(hardwareMap, driverGamepad, operatorGamepad, telemetry);
        driveTrain = new AutoFourWheelMecanumDriveTrain(hardwareMap, driverGamepad, telemetry, driverFeedback);

        // Create a trajectory (waypoints are in meters)
        // Max velocity of 1 m/s, max acceleration of 0.5 m/s^2
        config = new TrajectoryConfig(Tunables.MAX_VELOCITY_METERS_PER_SECOND, Tunables.MAX_ACCELERATION_METERS_PER_SECOND_SQ)
                .setKinematics(driveTrain.getDriveKinematics());

        robotDriver = createRobotDriver();
        register(driveTrain);

        setupWaypointsAndActions();

        setupCurrentState(getInitState());

        schedule(new SounderBotBaseRunCommand<AutoFourWheelMecanumDriveTrain>(driveTrain, telemetry, this::onEachCycle) {
            @Override
            public boolean isFinished() {
                return finished;
            }
        });
        timer.reset();
        addTelemetryLine("AutoOpMode init complete");
    }

    /**
     * method get called for each cycle, it call followTrajectory or performAction based on current state
     */
    private void onEachCycle() {
        switch (currentState) {
            case FOLLOWING_TRAJECTORY:
                followTrajectory();
                break;
            case PERFORMING_ACTION:
                performAction();
                break;
            case FINISHED:
                // do nothing
                break;
        }
    }

    /**
     * follow trajectory. This method read the odometer for current position, and get
     * desired position from trajectory, then pass the current and expected to updateTargetPose
     * to have the targetX, targetY and targetRotationSpeed, then it call driver to drive to the target
     * position. Then call driveTrain.updatePose to actually drive it.
     * <p>For each cycle, isTrajectoryComplete is called to have chance to finish the trajectory and
     * potentially switch to perform action mode</p>
     */
    private void followTrajectory() {
        double elapsedTime = timer.seconds();
        Pose2d currentPose = driveTrain.getCurrentPose();
        Pose2d desiredPose = currentTrajectory.sample(elapsedTime).poseMeters;
        double duration = currentTrajectory.getTotalTimeSeconds();

        updateTargetPose(currentPose, desiredPose);
        addTelemetryLine(String.format(Locale.getDefault(),"targetX = %f, targetY = %f, targetRotationSpeed = %f, trajectoriesRun = %d, duration = %f",  targetX, targetY, targetRotationSpeed, trajectoryCount-1, duration));
//        Log.i(LOG_TAG, String.format("targetX = %f, targetY = %f, targetRotationSpeed = %f", targetX, targetY, targetRotationSpeed));
//        RobotLog.i("targetX = %f, targetY = %f, targetRotationSpeed = %f", targetX, targetY, targetRotationSpeed);
//        System.out.printf("targetX = %f, targetY = %f, targetRotationSpeed = %f", targetX, targetY, targetRotationSpeed);
        robotDriver.drive(targetX, targetY, targetRotationSpeed);
        driveTrain.updatePose();
        if (isTrajectoryComplete(currentTrajectory, currentPose, elapsedTime)) {
            driveTrain.stop();
            setupCurrentState(State.PERFORMING_ACTION);
        }
    }

    /**
     * perform a blocking action, after it finished, switch to following trajectory mode
     */
    private void performAction() {
        currentAction.execute();
        addTelemetryLine("Action done");
        setupCurrentState(State.FOLLOWING_TRAJECTORY);
    }

    /**
     * This method will be called for each cycle to setup targetX, targetY, and targetRotationSpeed
     */
    protected abstract void updateTargetPose(Pose2d currentPose, Pose2d desiredPose);

    /**
     * This method should setup the @{code Stack<List<Pose2d>> wayPointsStack} and @{{@code Stack<RunCommand> actions}}
     * The amount of items in these 2 stacks does not need to be same, internal logic will make sure
     * the remaining trajectory or action will be executed.
     *
     * <p>Each trajectory is formed with list of waypoints,
     * each waypoint is an instance of Pose2d, contains data of x, y, and direction.
     * The follow trajectory is reaching the waypoint by order in a smooth path</p>
     */
    protected abstract void setupWaypointsAndActions();

    /**
     * this method to create a driver for drivetrain. default to RobotCentricDriver,
     * If want to drive by field centric, child class could override this method to return a new
     * instance of FieldCentricDriver
     */
    protected AutoRobotDriver createRobotDriver() {
        addTelemetryData("drive train class: %s", driveTrain.getClass().getName());
        return new RobotCentricDriver(driveTrain);
    }

    /**
     * Determine is the trajectory is completed or not. The trajectory is considered complete when all
     * following conditions are satisfied:
     * <li>eclipsed time is equal or greater than expected time of trajectory</li>
     * <li>the position error (x, y, and rotation) all within tolerance</li>
     * <p>Note: tolerance values should be tuned to fit the robot</p>
     */
    private boolean isTrajectoryComplete(Trajectory currentTrajectory, Pose2d currentPose, double elapsedTime) {
        Pose2d endPose = currentTrajectory.sample(elapsedTime).poseMeters;

        double xError = Math.abs(currentPose.getX() - endPose.getX());
        double yError = Math.abs(currentPose.getY() - endPose.getY());
        double rotationError = Math.abs(currentPose.getRotation().minus(endPose.getRotation()).getRadians());

        addTelemetryLine(String.format(Locale.getDefault(),"xError: %f (%f), yError: %f (%f), rotError: %f (%f)",
                xError, Tunables.X_TOLERANCE,
                yError, Tunables.Y_TOLERANCE,
                rotationError, Tunables.ROTATION_TOLERANCE));
//        Log.i(LOG_TAG, String.format("xError: %f (%f), yError: %f (%f), rotError: %f (%f)",
//                xError, Tunables.X_TOLERANCE,
//                yError, Tunables.Y_TOLERANCE,
//                rotationError, Tunables.ROTATION_TOLERANCE));
//
//        RobotLog.i("xError: %f (%f), yError: %f (%f), rotError: %f (%f)",
//                xError, Tunables.X_TOLERANCE,
//                yError, Tunables.Y_TOLERANCE,
//                rotationError, Tunables.ROTATION_TOLERANCE);
        return xError <=
                Tunables.X_TOLERANCE &&
                yError <= Tunables.Y_TOLERANCE &&
                rotationError <= Tunables.ROTATION_TOLERANCE;
    }

    /**
     * Define what is the initial state of the robot, the auto mode should start with
     * following trajectory or performing action
     */
    protected State getInitState() {
        return State.FOLLOWING_TRAJECTORY;
    }

    protected void resetControls() {
        timer.reset();
    }

    /**
     * set state of the robot. If no more instruction from current state stack, try to finish
     * all remaining items from other stack, if nothing left, set state to FINISHED.
     */
    private void setupCurrentState(State state) {
        if (currentState != null && currentState.equals(state)) {
            return;
        }
        addTelemetryData("Switching to %s mode", state);
        currentState = state;
        switch (currentState) {
            case FOLLOWING_TRAJECTORY:
                if (!wayPointsStack.isEmpty()) {
                    trajectoryCount += 1;
                    currentTrajectory = TrajectoryGenerator.generateTrajectory(wayPointsStack.pop(), config);
                    resetControls();
                } else if (!actions.isEmpty()) {
                    setupCurrentState(State.PERFORMING_ACTION);
                } else {
                    setupCurrentState(State.FINISHED);
                }
                break;
            case PERFORMING_ACTION:
                if (!actions.isEmpty()) {
                    currentAction = actions.pop();
                    resetControls();
                } else if (!wayPointsStack.isEmpty()) {
                    setupCurrentState(State.FOLLOWING_TRAJECTORY);
                } else {
                    setupCurrentState(State.FINISHED);
                }
                break;
            case FINISHED:
                resetControls();
                finished = true;
                break;
        }
    }

    protected void addTelemetryData(String caption, Object... values) {
//        RobotLog.i(TELEMETRY_CAPTION_PREFIX + caption, value);
        Log.i(LOG_TAG, String.format(TELEMETRY_CAPTION_PREFIX + caption, values));
        telemetry.addData(TELEMETRY_CAPTION_PREFIX + caption, values);
    }

    protected void addTelemetryLine(String line) {
//        RobotLog.i(TELEMETRY_CAPTION_PREFIX + line);
        Log.i(LOG_TAG, TELEMETRY_CAPTION_PREFIX + line);
        telemetry.addLine(TELEMETRY_CAPTION_PREFIX + line);
    }

    protected enum State {
        FOLLOWING_TRAJECTORY,
        PERFORMING_ACTION,
        FINISHED
    }
}