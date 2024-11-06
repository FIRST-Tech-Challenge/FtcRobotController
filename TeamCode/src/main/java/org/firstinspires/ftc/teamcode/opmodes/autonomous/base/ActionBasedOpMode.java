package org.firstinspires.ftc.teamcode.opmodes.autonomous.base;

import android.util.Log;

import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.command.SounderBotBaseRunCommand;
import org.firstinspires.ftc.teamcode.opmodes.OpModeTemplate;
import org.firstinspires.ftc.teamcode.opmodes.autonomous.driver.AutoRobotDriver;
import org.firstinspires.ftc.teamcode.opmodes.autonomous.driver.RobotCentricDriver;
import org.firstinspires.ftc.teamcode.opmodes.autonomous.movement.BaseMovement;
import org.firstinspires.ftc.teamcode.opmodes.autonomous.movement.ForwardMovement;
import org.firstinspires.ftc.teamcode.opmodes.autonomous.movement.RotationMovement;
import org.firstinspires.ftc.teamcode.opmodes.autonomous.movement.StrafeMovement;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.AutoFourWheelMecanumDriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.feedback.DriverFeedback;

import java.util.LinkedList;
import java.util.Locale;
import java.util.Queue;

/**
 * <p>This is main class for auto op mode. It register the callback for each control hub scan cycle
 * (typically 50ms per scan), for each cycle, it will follow trajectory or perform action based on current state</p>
 *
 */
public abstract class ActionBasedOpMode extends OpModeTemplate {

    private static final String LOG_TAG = ActionBasedOpMode.class.getName();
    private final String TELEMETRY_CAPTION_PREFIX = "[v4-" + getClass().getSimpleName() + "] ";
    private final ElapsedTime timer = new ElapsedTime();
    protected AutoFourWheelMecanumDriveTrain driveTrain;

    protected DriverFeedback driverFeedback;

    protected Queue<Queue<BaseMovement>> movementQueue = new LinkedList<>();
    protected Queue<RunCommand> actions = new LinkedList<>();
    private Queue<BaseMovement> currentMovements;
    private BaseMovement currentMove;
    private RunCommand currentAction;
    private AutoRobotDriver robotDriver;
    private State currentState;
    private boolean finished = false;

    @Override
    public void initialize() {
        driverFeedback = new DriverFeedback(hardwareMap, driverGamepad, operatorGamepad, telemetry);
        driveTrain = new AutoFourWheelMecanumDriveTrain(hardwareMap, driverGamepad, telemetry, driverFeedback);

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
            case PERFORM_MOVEMENT:
                performMovements();
                break;
            case PERFORMING_ACTION:
                performAction();
                break;
            case FINISHED:
                // do nothing
                break;
        }
    }

    private void performMovements() {
        if (currentMove == null || currentMove.isFinished()) {
            if (currentMovements.isEmpty()) {
                setupCurrentState(State.PERFORMING_ACTION);
                return;
            }
            sleep(250);
            currentMove = currentMovements.poll();
            assert currentMove != null;
//            driveTrain.resetOdo();
            addTelemetryLine(String.format("Switched to movement %s", currentMove.getClass().getSimpleName()));
            if (currentMove instanceof ForwardMovement) {
                currentMove.start(driveTrain.getCurrentPose().getX(), robotDriver, () -> driveTrain.getCurrentPose());
            } else if (currentMove instanceof StrafeMovement) {
                currentMove.start(driveTrain.getCurrentPose().getY(), robotDriver, () -> driveTrain.getCurrentPose());
            } else if (currentMove instanceof RotationMovement) {
                currentMove.start(driveTrain.getCurrentPose().getRotation().getRadians(), robotDriver,  () -> driveTrain.getCurrentPose());
            }
        } else {
            currentMove.onEachCycle();
        }
        driveTrain.updatePose();
        Pose2d current = driveTrain.getCurrentPose();

        addTelemetryLine(String.format(Locale.getDefault(),"%s: x=%f, y=%f, rotation=%f", currentMove.getClass().getSimpleName(), current.getX(), current.getY(), current.getRotation().getRadians()));
    }

    /**
     * perform a blocking action, after it finished, switch to following trajectory mode
     */
    private void performAction() {
        currentAction.execute();
        addTelemetryLine("Action done");
        setupCurrentState(State.PERFORM_MOVEMENT);
    }

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
     * Define what is the initial state of the robot, the auto mode should start with
     * following trajectory or performing action
     */
    protected State getInitState() {
        return State.PERFORM_MOVEMENT;
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
        sleep(100);
        driveTrain.resetOdo();

        addTelemetryData("Switching to %s mode", state);
        currentState = state;
        switch (currentState) {
            case PERFORM_MOVEMENT:
                if (!movementQueue.isEmpty()) {
                    currentMovements = movementQueue.poll();
                    resetControls();
                } else if (!actions.isEmpty()) {
                    setupCurrentState(State.PERFORMING_ACTION);
                } else {
                    setupCurrentState(State.FINISHED);
                }
                break;
            case PERFORMING_ACTION:
                if (!actions.isEmpty()) {
                    currentAction = actions.poll();
                    resetControls();
                } else if (!movementQueue.isEmpty()) {
                    setupCurrentState(State.PERFORM_MOVEMENT);
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
        Log.i(LOG_TAG, String.format(TELEMETRY_CAPTION_PREFIX + caption, values));
        telemetry.addData("AutoMpMode", String.format(TELEMETRY_CAPTION_PREFIX + caption, values));
    }

    protected void addTelemetryLine(String line) {
        Log.i(LOG_TAG, TELEMETRY_CAPTION_PREFIX + line);
        telemetry.addLine(TELEMETRY_CAPTION_PREFIX + line);
    }

    protected enum State {
        PERFORM_MOVEMENT,
        PERFORMING_ACTION,
        FINISHED
    }
}