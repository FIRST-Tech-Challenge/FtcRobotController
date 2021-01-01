package org.firstinspires.ftc.teamcode.team10515.subsystems;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.lib.annotations.PIDSVA;
import org.firstinspires.ftc.teamcode.lib.control.ControlConstants;
import org.firstinspires.ftc.teamcode.lib.drivers.RevMotor;
import org.firstinspires.ftc.teamcode.lib.drivers.RevServo;
import org.firstinspires.ftc.teamcode.lib.motion.IMotionProfile;
import org.firstinspires.ftc.teamcode.lib.motion.ResidualVibrationReductionMotionProfilerGenerator;
import org.firstinspires.ftc.teamcode.lib.util.Time;
import org.firstinspires.ftc.teamcode.lib.util.TimeProfiler;
import org.firstinspires.ftc.teamcode.lib.util.TimeUnits;
import org.firstinspires.ftc.teamcode.team10515.control.StackTracker;
import org.firstinspires.ftc.teamcode.team10515.states.FlickerStateMachine;
import org.firstinspires.ftc.teamcode.team10515.states.FeederExtensionStateMachine;
import org.firstinspires.ftc.teamcode.team10515.states.FeederStateMachine;
import org.firstinspires.ftc.teamcode.team10515.states.FeederStoneGripperStateMachine;
import org.firstinspires.ftc.teamcode.team10515.states.FlywheelStateMachine;
import org.firstinspires.ftc.teamcode.team10515.states.VirtualFourBarStateMachine;

import java.util.function.DoubleSupplier;

@PIDSVA(name = "Extend",
        P = 0.4d,
        I = 0d,
        D = 0d,
        S = 0.14d,
        V = (1 - 0.14) / 15d,
        A = 0d
)

@PIDSVA(name = "Retract",
        P = 0.0025d,
        I = 0d,
        D = 0d,
        S = 0.06d,
        V = 1 / 55d,
        A = 0d
)
public class Feeder implements ISubsystem<FeederStateMachine, FeederStateMachine.State> {
    private static final Time STONE_IN_ROBOT_TIME_THRESHOLD = new Time(0.6d, TimeUnits.SECONDS);
    private static final double STONE_IN_ROBOT_DISTANCE_THRESHOLD = 4d;
    private static final ControlConstants EXTEND_CONTROL_CONSTANTS;
    private static final ControlConstants RETRACT_CONTROL_CONSTANTS;

    private static FeederStateMachine feederStateMachine;
    private static FeederExtensionStateMachine feederExtensionStateMachine;
    private static VirtualFourBarStateMachine virtualFourBarStateMachine;
    private static FeederStoneGripperStateMachine feederStoneGripperStateMachine;
    private static FlickerStateMachine FlickerStateMachine;
    private static FlywheelStateMachine flywheelStateMachine;
    private static StackTracker stackTracker;
    private RevMotor leftExtension;
    private RevMotor rightExtension;
    private RevServo leftFourBarJoint;
    private RevServo rightFourBarJoint;
    private RevServo stoneGripper;
    private RevServo capstoneArm;

    private Rev2mDistanceSensor stoneDetector;
    private TimeProfiler timeProfilerStoneDetection;

    private static IMotionProfile extensionProfile = null;
    private static double setpoint = 0d;
    private static double desiredSetpoint = 0d;
    private double lastError;
    private double runningSum;

    private static DoubleSupplier manualControlExtension;


    private boolean isDeliveryMode = false;

    static {
        new Thread(ResidualVibrationReductionMotionProfilerGenerator::init).start();
        PIDSVA[] controllers = Feeder.class.getAnnotationsByType(PIDSVA.class);
        if(controllers.length == 2) {
            PIDSVA extendController;
            PIDSVA retractController;
            if(controllers[0].name().equals(FeederExtensionStateMachine.State.EXTEND.getName())) {
                extendController  = controllers[0];
                retractController = controllers[1];
            } else {
                extendController  = controllers[1];
                retractController = controllers[0];
            }

            EXTEND_CONTROL_CONSTANTS = new ControlConstants(
                extendController.P(), extendController.I(), extendController.D(),
                    extendController.S(), extendController.V(), extendController.A()
            );

            RETRACT_CONTROL_CONSTANTS = new ControlConstants(
                retractController.P(), retractController.I(), retractController.D(),
                    retractController.S(), retractController.V(), retractController.A()
            );
        } else {
            EXTEND_CONTROL_CONSTANTS  = new ControlConstants();
            RETRACT_CONTROL_CONSTANTS = new ControlConstants();
        }

        FeederExtensionStateMachine.setRunExtension((setpoint) -> {
            if(getExtensionProfile() != null) {
                getExtensionProfile().start();
                Feeder.setpoint = setpoint;
            }
        });
    }

    public Feeder(FlywheelStateMachine flywheelStateMachine, StackTracker stackTracker, RevMotor leftExtension,
                  RevMotor rightExtension, RevServo leftFourBarJoint, RevServo rightFourBarJoint,
                  RevServo stoneGripper, RevServo capstoneArm, Rev2mDistanceSensor stoneDetector) {
        setFeederStateMachine(new FeederStateMachine());
        setFeederExtensionStateMachine(new FeederExtensionStateMachine(this));
        setVirtualFourBarStateMachine(new VirtualFourBarStateMachine());
        setFeederStoneGripperStateMachine(new FeederStoneGripperStateMachine());
        setFlickerStateMachine(new FlickerStateMachine());
        setFlywheelStateMachine(flywheelStateMachine);
        setStackTracker(stackTracker);
        setLeftExtension(leftExtension);
        setRightExtension(rightExtension);
        setLeftFourBarJoint(leftFourBarJoint);
        setRightFourBarJoint(rightFourBarJoint);
        setStoneGripper(stoneGripper);
        setCapstoneArm(capstoneArm);
        setStoneDetector(stoneDetector);
        getStoneDetector().initialize();
        setTimeProfilerStoneDetection(new TimeProfiler(false));
        setLastError(0d);
        resetRunningSum();
    }

    public void resetRunningSum() {
        setRunningSum(0d);
    }

    @Override
    public FeederStateMachine getStateMachine() {
        return feederStateMachine;
    }

    @Override
    public FeederStateMachine.State getState() {
        return getStateMachine().getState();
    }

    @Override
    public void start() {
        getStoneDetector().resetDeviceConfigurationForOpMode();
        getTimeProfilerStoneDetection().start();
    }

    @Override
    public void stop() {
        getLeftExtension().setPower(0d);
        getRightExtension().setPower(0d);
        //Yogesh - commented close
        //getStoneDetector().close();
    }

    @Override
    public String getName() {
        return "Feeder";
    }

    @Override
    public void writeToTelemetry(Telemetry telemetry) {

    }

    @Override
    public void update(double dt) {
        if(getTimeProfilerStoneDetection().getDeltaTime(TimeUnits.SECONDS, false) > 200d) {
            getTimeProfilerStoneDetection().start();
        }

        getStateMachine().update(dt);
        getFeederExtensionStateMachine().update(dt);
        getVirtualFourBarStateMachine().update(dt);
        getFeederStoneGripperStateMachine().update(dt);
        getFlickerStateMachine().update(dt);

        double error                = getSetpoint() - getLeftExtension().getPosition();
        double setpointVelocity     = 0d;
        double setpointAcceleration = 0d;
        if(getExtensionProfile() != null && !getExtensionProfile().isDone()) {
            setpointVelocity     = getExtensionProfile().getVelocity();
            setpointAcceleration = getExtensionProfile().getAcceleration();
        }

        setRunningSum(getRunningSum() + error * dt);
        double output = 0d;
        if(getManualControlExtension() != null && getVirtualFourBarStateMachine().hasReachedStateGoal(VirtualFourBarStateMachine.State.STACK)) {
            //Manual control
            output = getManualControlExtension().getAsDouble();
            if(output > 0d) {
                getFeederExtensionStateMachine().updateState(FeederExtensionStateMachine.State.EXTEND);
            }

            output += getExtendControlConstants().kS();
        } else if(getFeederExtensionStateMachine().getState().equals(FeederExtensionStateMachine.State.EXTEND)) {
            output = getExtendControlConstants().getOutput(dt, error, getLastError(), getRunningSum(), setpointVelocity, setpointAcceleration, false);
            output += getExtendControlConstants().kS();
            if (closeToSetpoint(1 / 4d)) {
                //getVirtualFourBarStateMachine().updateState(VirtualFourBarStateMachine.State.STACK);
            }
        } else {
            output = getRetractControlConstants().getOutput(dt, error, getLastError(), getRunningSum(), setpointVelocity, setpointAcceleration, true);
            if (!isDeliveryMode) {
                if (hasStoneInRobot()) {
                    if (getTimeProfilerStoneDetection().getDeltaTime(TimeUnits.SECONDS, false) > getStoneInRobotTimeThreshold().getTimeValue(TimeUnits.SECONDS)) {
                        getFeederStoneGripperStateMachine().updateState(FeederStoneGripperStateMachine.State.GRIP);
                        getFlywheelStateMachine().updateState(FlywheelStateMachine.State.IDLE);
                    }
                } else {
                    getTimeProfilerStoneDetection().update(true);
                    if (!getVirtualFourBarStateMachine().getDesiredState().equals(VirtualFourBarStateMachine.State.STACK)) {
                        getFeederStoneGripperStateMachine().updateState(FeederStoneGripperStateMachine.State.NO_GRIP);
                    }
                }
            }
        }

        setLastError(error);

        final double kP = 0.001d;
        double relativeError = getLeftExtension().getPosition() - getRightExtension().getPosition();
        double relativeOutput = kP * relativeError;

        getLeftExtension().setPower(output);
        getRightExtension().setPower(output + relativeOutput);
        getLeftFourBarJoint().setPosition(getVirtualFourBarStateMachine().getState().getLeftPosition());
        getRightFourBarJoint().setPosition(getVirtualFourBarStateMachine().getState().getRightPosition());
        getStoneGripper().setPosition(getFeederStoneGripperStateMachine().getState().getPosition());
        getFlickerArm().setPosition(getFlickerStateMachine().getState().getPosition());
    }

    public void toggleVirtualFourBar() {
        getVirtualFourBarStateMachine().updateState(
                getVirtualFourBarStateMachine().getState().equals(VirtualFourBarStateMachine.State.STACK) ?
                        VirtualFourBarStateMachine.State.IN_ROBOT : VirtualFourBarStateMachine.State.STACK);
    }

    public void toggleStoneGripper() {
        getFeederStoneGripperStateMachine().updateState(
                getFeederStoneGripperStateMachine().getState().equals(FeederStoneGripperStateMachine.State.GRIP) ?
                        FeederStoneGripperStateMachine.State.NO_GRIP : FeederStoneGripperStateMachine.State.GRIP);
    }

    public boolean hasStoneInRobot() {
        return getStoneDetector().getDistance(DistanceUnit.INCH) <= getStoneInRobotDistanceThreshold();
    }

    public void extend() {
        resetRunningSum();
        setSetpoint(getStackTracker().getExtensionHeight());
    }

    public void retract() {
        getVirtualFourBarStateMachine().updateState(VirtualFourBarStateMachine.State.IN_ROBOT);
        resetRunningSum();
        setSetpoint(0d);
    }

    public boolean closeToSetpoint(double threshold) {
        return Math.abs(getSetpoint() - getLeftExtension().getPosition()) <= threshold;
    }

    public static void setFeederStateMachine(FeederStateMachine feederStateMachine) {
        Feeder.feederStateMachine = feederStateMachine;
    }

    public static FeederExtensionStateMachine getFeederExtensionStateMachine() {
        return feederExtensionStateMachine;
    }

    public static void setFeederExtensionStateMachine(FeederExtensionStateMachine feederExtensionStateMachine) {
        Feeder.feederExtensionStateMachine = feederExtensionStateMachine;
    }

    public static VirtualFourBarStateMachine getVirtualFourBarStateMachine() {
        return virtualFourBarStateMachine;
    }

    public static void setVirtualFourBarStateMachine(VirtualFourBarStateMachine virtualFourBarStateMachine) {
        Feeder.virtualFourBarStateMachine = virtualFourBarStateMachine;
    }

    public static FeederStoneGripperStateMachine getFeederStoneGripperStateMachine() {
        return feederStoneGripperStateMachine;
    }

    public static void setFeederStoneGripperStateMachine(FeederStoneGripperStateMachine feederStoneGripperStateMachine) {
        Feeder.feederStoneGripperStateMachine = feederStoneGripperStateMachine;
    }

    public RevMotor getLeftExtension() {
        return leftExtension;
    }

    public void setLeftExtension(RevMotor leftExtension) {
        this.leftExtension = leftExtension;
    }

    public RevMotor getRightExtension() {
        return rightExtension;
    }

    public void setRightExtension(RevMotor rightExtension) {
        this.rightExtension = rightExtension;
    }

    public RevServo getLeftFourBarJoint() {
        return leftFourBarJoint;
    }

    public void setLeftFourBarJoint(RevServo leftFourBarJoint) {
        this.leftFourBarJoint = leftFourBarJoint;
    }

    public RevServo getRightFourBarJoint() {
        return rightFourBarJoint;
    }

    public void setRightFourBarJoint(RevServo rightFourBarJoint) {
        this.rightFourBarJoint = rightFourBarJoint;
    }

    public RevServo getStoneGripper() {
        return stoneGripper;
    }

    public void setStoneGripper(RevServo stoneGripper) {
        this.stoneGripper = stoneGripper;
    }

    public static double getSetpoint() {
        return setpoint;
    }

    public void setSetpoint(double setpoint) {
        setDesiredSetpoint(setpoint);
        if(setpoint != getSetpoint() && (getExtensionProfile() == null || getExtensionProfile().isDone())) {
            if(setpoint != 0d) {
                //Extending
                getFeederExtensionStateMachine().updateState(FeederExtensionStateMachine.State.EXTEND);
                setExtensionProfile(getStackTracker().motionProfilerSetpoints(true));
            } else {
                //Retracting
                getFeederExtensionStateMachine().updateState(FeederExtensionStateMachine.State.RETRACT);
                //setExtensionProfile(getStackTracker().motionProfilerSetpoints(false));
                setExtensionProfile(new ResidualVibrationReductionMotionProfilerGenerator(
                        getLeftExtension().getPosition(), -getLeftExtension().getPosition(), 25d, 50d
                ));
            }
        }
    }

    public RevServo getFlickerArm() {
        return capstoneArm;
    }

    public void setCapstoneArm(RevServo capstoneArm) {
        this.capstoneArm = capstoneArm;
    }

    public static FlickerStateMachine getFlickerStateMachine() {
        return FlickerStateMachine;
    }

    public static void setFlickerStateMachine(FlickerStateMachine FlickerStateMachine) {
        Feeder.FlickerStateMachine = FlickerStateMachine;
    }

    public static IMotionProfile getExtensionProfile() {
        return extensionProfile;
    }

    public static void setExtensionProfile(IMotionProfile extensionProfile) {
        Feeder.extensionProfile = extensionProfile;
    }

    public static ControlConstants getExtendControlConstants() {
        return EXTEND_CONTROL_CONSTANTS;
    }

    public static ControlConstants getRetractControlConstants() {
        return RETRACT_CONTROL_CONSTANTS;
    }

    public double getLastError() {
        return lastError;
    }

    public void setLastError(double lastError) {
        this.lastError = lastError;
    }

    public double getRunningSum() {
        return runningSum;
    }

    public void setRunningSum(double runningSum) {
        this.runningSum = runningSum;
    }

    public Rev2mDistanceSensor getStoneDetector() {
        return stoneDetector;
    }

    public void setStoneDetector(Rev2mDistanceSensor stoneDetector) {
        this.stoneDetector = stoneDetector;
    }

    public static StackTracker getStackTracker() {
        return stackTracker;
    }

    public static void setStackTracker(StackTracker stackTracker) {
        Feeder.stackTracker = stackTracker;
    }

    public TimeProfiler getTimeProfilerStoneDetection() {
        return timeProfilerStoneDetection;
    }

    public void setTimeProfilerStoneDetection(TimeProfiler timeProfilerStoneDetection) {
        this.timeProfilerStoneDetection = timeProfilerStoneDetection;
    }

    public void setDeliveryMode(boolean deliveryMode){
        this.isDeliveryMode = deliveryMode;
    }

    public boolean getDeliveryMode(){ return isDeliveryMode; }

    public static Time getStoneInRobotTimeThreshold() {
        return STONE_IN_ROBOT_TIME_THRESHOLD;
    }

    public static double getStoneInRobotDistanceThreshold() {
        return STONE_IN_ROBOT_DISTANCE_THRESHOLD;
    }

    public static double getDesiredSetpoint() {
        return desiredSetpoint;
    }

    public static void setDesiredSetpoint(double desiredSetpoint) {
        Feeder.desiredSetpoint = desiredSetpoint;
    }

    public static FlywheelStateMachine getFlywheelStateMachine() {
        return flywheelStateMachine;
    }

    public static void setFlywheelStateMachine(FlywheelStateMachine flywheelStateMachine) {
        Feeder.flywheelStateMachine = flywheelStateMachine;
    }

    public static DoubleSupplier getManualControlExtension() {
        return manualControlExtension;
    }

    public static void setManualControlExtension(DoubleSupplier manualControlExtension) {
        Feeder.manualControlExtension = manualControlExtension;
    }
}
