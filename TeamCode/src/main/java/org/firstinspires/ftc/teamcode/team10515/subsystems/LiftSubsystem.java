package org.firstinspires.ftc.teamcode.team10515.subsystems;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.lib.annotations.PIDSVA;
import org.firstinspires.ftc.teamcode.lib.control.ControlConstants;
import org.firstinspires.ftc.teamcode.lib.drivers.RevMotor;
import org.firstinspires.ftc.teamcode.lib.motion.IMotionProfile;
import org.firstinspires.ftc.teamcode.team10515.DbgLog;
import org.firstinspires.ftc.teamcode.team10515.states.LiftStateMachine;

@PIDSVA(name = "Extend",
        P = 0.4d,
        I = 0d,
        D = 0.00d,
        S = 0.05d,
        V = (1 - 0.05) / 15d,
        A = 0d
)

@PIDSVA(name = "Retract",
        P = 0.08d,
        I = 0d,
        D = 0d,
        S = 0.05d,
        V = 1 / 55d,
        A = 0d
)
public class LiftSubsystem implements ISubsystem<LiftStateMachine, LiftStateMachine.State> {
    private static final ControlConstants EXTEND_CONTROL_CONSTANTS;
    private static final ControlConstants RETRACT_CONTROL_CONSTANTS;

    private static LiftStateMachine liftStateMachine;
    private RevMotor liftL;
    private RevMotor liftR;

    private static IMotionProfile extensionProfile = null;
    private static double setpoint = 0d;
    private static double desiredSetpoint = 0d;
    private double lastError;
    private double runningSum;

    static {
        //new Thread(ResidualVibrationReductionMotionProfilerGenerator::init).start();
        PIDSVA[] controllers = LiftSubsystem.class.getAnnotationsByType(PIDSVA.class);
        if(controllers.length == 2) {
            PIDSVA extendController;
            PIDSVA retractController;
            if(controllers[0].name().equals(LiftStateMachine.State.EXTEND.getName())) {
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

//        LiftStateMachine.setRunExtension((setpoint) -> {
//            if(getExtensionProfile() != null) {
//                getExtensionProfile().start();
//                LiftSubsystem.setpoint = setpoint;
//            }
//        });
    }

    public LiftSubsystem(RevMotor liftL, RevMotor liftR) {
        setLiftStateMachine(new LiftStateMachine(this));
        setLiftL(liftL);
        setLiftR(liftR);
    //    getLiftL().setEncoderTicksPerInch(231.157895d);
    //    getLiftR().setEncoderTicksPerInch(231.157895d);
        setLastError(0d);
        resetRunningSum();
    }

    public void resetRunningSum() {
        setRunningSum(0d);
    }

    @Override
    public LiftStateMachine getStateMachine() {
        return liftStateMachine;
    }

    @Override
    public LiftStateMachine.State getState() {
        return getStateMachine().getState();
    }

    @Override
    public void start() {
    }

    @Override
    public void stop() {
        getLiftL().setPower(0d);
        getLiftR().setPower(0d);
        getLiftL().resetEncoder();
        getLiftR().resetEncoder();
        setSetpoint(0d);
        setDesiredSetpoint(0d);
    }

    @Override
    public String getName() {
        return "Lift";
    }

    @Override
    public void writeToTelemetry(Telemetry telemetry) {

    }

    @Override
    public void update(double dt) {
        //getStateMachine().update(dt);
        getLiftStateMachine().update(dt);

        double error                = getSetpoint() - getLiftL().getPosition();
        double setpointVelocity     = 0d;
        double setpointAcceleration = 0d;
        if(getExtensionProfile() != null && !getExtensionProfile().isDone()) {
            setpointVelocity     = getExtensionProfile().getVelocity();
            setpointAcceleration = getExtensionProfile().getAcceleration();
        }

        DbgLog.msg("Error: " + String.valueOf(error));

        setRunningSum(getRunningSum() + error * dt);
        double output;
        if(getLiftStateMachine().getState().equals(LiftStateMachine.State.EXTEND)) { //changed from getstate to getdesiredstate
            output = getExtendControlConstants().getOutput(dt, error, getLastError(), getRunningSum(), setpointVelocity, setpointAcceleration, false);
            output += getExtendControlConstants().kS();
        }
        else {
            output = getRetractControlConstants().getOutput(dt, error, getLastError(), getRunningSum(), setpointVelocity, setpointAcceleration, true);
        }

        DbgLog.msg("Output: " + String.valueOf(output));

        setLastError(error);

        final double kP = 0.001d;
        double relativeError = getLiftL().getPosition() - getLiftR().getPosition();;
        double relativeOutput = kP * relativeError;

        getLiftL().setPower(output);
        getLiftR().setPower(output + relativeOutput);
    }

    public void extend(Double set) {
        resetRunningSum();
        setSetpoint(set);
    }

    public void retract() {
        resetRunningSum();
        setSetpoint(0d);
        setDesiredSetpoint(0d);
    }

    public boolean closeToSetpoint(double threshold) {
        return Math.abs(getSetpoint() - getLiftL().getPosition()) <= threshold;
    }

    public static LiftStateMachine getLiftStateMachine() {
        return liftStateMachine;
    }

    public static void setLiftStateMachine(LiftStateMachine liftStateMachine) {
        LiftSubsystem.liftStateMachine = liftStateMachine;
    }

    public RevMotor getLiftL() {
        return liftL;
    }

    public void setLiftL(RevMotor liftL) {
        this.liftL = liftL;
    }

    public RevMotor getLiftR() {
        return liftR;
    }

    public void setLiftR(RevMotor liftR) {
        this.liftR = liftR;
    }

    public static double getSetpoint() {
        return setpoint;
    }

    public void setSetpoint(double setpoint) {
        setDesiredSetpoint(setpoint);
        if(setpoint != getSetpoint() && (getExtensionProfile() == null || getExtensionProfile().isDone())) {
            if(setpoint != 0d) {
                //Extending
                getLiftStateMachine().updateState(LiftStateMachine.State.EXTEND);
                this.setpoint = setpoint;
            } else {
                //Retracting
                getLiftStateMachine().updateState(LiftStateMachine.State.RETRACT);
                this.setpoint = setpoint;
                //setExtensionProfile(new ResidualVibrationReductionMotionProfilerGenerator(
                //        getLiftL().getPosition(), -getLiftL().getPosition(), 25d, 50d
                //));
            }
        }
    }

    public static IMotionProfile getExtensionProfile() {
        return extensionProfile;
    }

    public static void setExtensionProfile(IMotionProfile extensionProfile) {
        LiftSubsystem.extensionProfile = extensionProfile;
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

     public static double getDesiredSetpoint() {
        return desiredSetpoint;
    }

    public static void setDesiredSetpoint(double desiredSetpoint) {
        LiftSubsystem.desiredSetpoint = desiredSetpoint;
    }
}
