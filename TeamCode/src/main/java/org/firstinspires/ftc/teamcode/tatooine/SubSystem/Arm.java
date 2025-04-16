package org.firstinspires.ftc.teamcode.tatooine.SubSystem;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.tatooine.utils.DebugUtils;
import org.firstinspires.ftc.teamcode.tatooine.utils.PIDFController;
import org.firstinspires.ftc.teamcode.tatooine.utils.mathUtil.MathUtil;
import org.firstinspires.ftc.teamcode.tatooine.utils.unit.UnitConverter;
import org.firstinspires.ftc.teamcode.tatooine.utils.unit.unit;

@Config
public class Arm {

    private DcMotorEx angleLeft = null;  // Left motor controlling angle
    private DcMotorEx angleRight = null; // Right motor controlling angle

    private double startAngle = -7; // Initial starting angle of the arm

    private double ANGLE_OFFSET = 90;   // Angle offset for the arm

    private DigitalChannel limitSwitch = null; // Limit switch to detect arm position limits

    private final double ANGLE_AMP_LIMIT = 5000; // Maximum allowed amperage for angle motors


    // Extend system variables
    private DcMotorEx extendLeft = null; // Left motor for extension
    private DcMotorEx extendRight = null; // Right motor for extension

    private final double EXTEND_UP_LIMIT_AMPS = 0; // Upper amperage limit for extend motors
    private final double EXTEND_DOWN_LIMIT_AMPS = 10000000; // Lower amperage limit for extend motors
    public static double MIN_EXTEND = 37.0; // Minimum extension value
    public static double MAX_EXTEND = 62; // Maximum extension value

    public static double LIMIT = UnitConverter.convert(42.0, unit.INCHES, unit.CM) - 20.0 - 5.0; // Conversion for limit distance

    private PIDFController anglePID = new PIDFController(0.06, 0.0, 0.004, 0); // PID controller for angle movement
    private PIDFController extendPID = new PIDFController(0.2, 0.00001, 0.0, 0); // PID controller for extension movement

    private final double ANGLE_CPR = 28.0 * 100.0 * (34.0 / 16.0); // Counts per revolution for angle motors
    private final double EXTEND_CPR = 537.7; // Counts per revolution for extension motors

    private final double ANGLE_TIMEOUT = 2.5; // Timeout for angle movement
    private final double EXTEND_TIMEOUT = 4; // Timeout for extension movement

    private final double ANGLE_TOLERANCE = 1.5; // Tolerance for angle movement
    private final double EXTEND_TOLERANCE = 1.5; // Tolerance for extension movement

    private final double LIMIT_TOLERANCE = 0.0; // Tolerance for limit switch reading

    private final double SPOOL_DIM = 3.82; // Spool dimensions for extension calculation

    private double lastAngle = 0.0; // Last recorded angle for force calculation81.68
    private double lastF = 0.0; // Last recorded force value

    private String SUBSYSTEM_NAME = "Arm"; // Name of the subsystem

    private boolean IS_DEBUG_MODE = false; // Flag to indicate if debug mode is enabled

    private OpMode opMode = null; // OpMode reference

    public static double KF = 0.16; // Constant factor for force calculation

    public static double KP = 0.7;

    public static double KI = 0;

    public static double KD = 0;

    private double lastPower = 0;

    // Constructor for the Arm system, passing opMode and debug flag
    public Arm(OpMode opMode, boolean isDebug) {
        IS_DEBUG_MODE = isDebug; // Set debug mode flag

        // Get hardware components from opMode
        angleRight = opMode.hardwareMap.get(DcMotorEx.class, "AR");
        angleLeft = opMode.hardwareMap.get(DcMotorEx.class, "AL");
        extendRight = opMode.hardwareMap.get(DcMotorEx.class, "ER");
        extendLeft = opMode.hardwareMap.get(DcMotorEx.class, "EL");
        limitSwitch = opMode.hardwareMap.get(DigitalChannel.class, "LS");

        this.opMode = opMode; // Set opMode

        init(); // Initialize hardware and settings
    }

    // Initialize the arm motors and reset encoders
    public void init() {
        // Set motor directions for angle and extension motors
        angleLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        angleRight.setDirection(DcMotorSimple.Direction.REVERSE);
        extendLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        extendRight.setDirection(DcMotorSimple.Direction.FORWARD);

        // Set zero power behavior to BRAKE for all motors to hold their position when no power is applied
        angleRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        angleLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        extendRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        extendLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set timeouts and tolerances for PID controllers
        anglePID.setTimeout(ANGLE_TIMEOUT);
        extendPID.setTimeout(EXTEND_TIMEOUT);
        anglePID.setTolerance(ANGLE_TOLERANCE);
        extendPID.setTolerance(EXTEND_TOLERANCE);

        resetEncoders(); // Reset encoders for all motors

        // Debug logging for initialization
        DebugUtils.logDebugMessage(opMode.telemetry, IS_DEBUG_MODE, SUBSYSTEM_NAME, "Arm Initialized");
    }

    // Reset all encoders for the arm system
    public void resetEncoders() {
        resetExtendEncoders(); // Reset extension motors encoders
        resetAngleEncoders(); // Reset angle motors encoders
    }

    // Reset encoders for extension motors
    public void resetExtendEncoders() {
        extendLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extendLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        extendRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extendRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    // Reset encoders for angle motors
    public void resetAngleEncoders() {
        angleLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        angleLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        angleRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        angleRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    // Get the current angle of the left arm motor
    public double getLeftAngle() {
        return MathUtil.convertTicksToDegrees(ANGLE_CPR, angleRight.getCurrentPosition());
    }

    // Get the current angle of the right arm motor
    public double getRightAngle() {
        return MathUtil.convertTicksToDegrees(ANGLE_CPR, angleLeft.getCurrentPosition());
    }

    // Get the average angle of the arm
    public double getAngle() {
        double angle = ((getRightAngle() + getLeftAngle()) / 2) + startAngle;
        DebugUtils.logDebug(opMode.telemetry, IS_DEBUG_MODE, SUBSYSTEM_NAME, "angle", angle);
        return angle;
    }

    // Get the current extension of the left motor
    public double getLeftExtend() {
        return MathUtil.convertTicksToDistance(EXTEND_CPR, SPOOL_DIM, extendLeft.getCurrentPosition());
    }

    // Get the current extension of the right motor
    public double getRightExtend() {
        return MathUtil.convertTicksToDistance(EXTEND_CPR, SPOOL_DIM, extendRight.getCurrentPosition());
    }

    // Get the average extension of the arm
    public double getExtend() {
        DebugUtils.logDebug(opMode.telemetry, IS_DEBUG_MODE, SUBSYSTEM_NAME, "extend", getRightExtend());
        return getRightExtend();
    }

    // Get the current current draw (in milliamps) for the left angle motor
    public double getCurrentAngleLeft() {
        return angleLeft.getCurrent(CurrentUnit.MILLIAMPS);
    }

    // Get the current current draw (in milliamps) for the right angle motor
    public double getCurrentAngleRight() {
        return angleRight.getCurrent(CurrentUnit.MILLIAMPS);
    }

    // Get the average current draw for the angle motors
    public double getCurrentAngle() {
        return (getCurrentAngleRight() + getCurrentAngleLeft()) / 2;
    }

    // Get the current current draw (in milliamps) for the left extension motor
    public double getCurrentExtendLeft() {
        return extendLeft.getCurrent(CurrentUnit.MILLIAMPS);
    }

    // Get the current current draw (in milliamps) for the right extension motor
    public double getCurrentExtendRight() {
        return extendRight.getCurrent(CurrentUnit.MILLIAMPS);
    }

    // Get the average current draw for the extension motors
    public double getCurrentExtend() {
        return getCurrentExtendRight();
    }

    public static double getMaxExtend() {
        return MAX_EXTEND;
    }

    public static double getMinExtend() {
        return MIN_EXTEND;
    }

    public void setStartAngle(double startAngle) {
        this.startAngle = startAngle;
    }

    // Calculate the force based on the current arm angle
    public double calculateF() {
        double currentAngle = getAngle();
        if (Math.abs(currentAngle - lastAngle) > 0.1) {
            lastF = (Math.cos(Math.toRadians(currentAngle)) * KF) * ((MIN_EXTEND + getExtend()) / MIN_EXTEND);
            lastAngle = currentAngle;
        }
        return lastF;
    }


    // Set power to the angle motors
    public void setAnglePower(double power) {

        // Prevent movement if the current angle exceeds the amperage limit
        if (getCurrentAngle() >= ANGLE_AMP_LIMIT * ((MIN_EXTEND + getExtend()) / MIN_EXTEND)){
            if (power > 0 && MathUtil.inTolerance(lastPower, power, 0.05))
            {
                resetAngleEncoders();
                startAngle = ANGLE_OFFSET;
            }
            power = 0;

        }
        if ((getAngle() < -4 && power < 0) || (getAngle() > 90 && power > 0)){
            power = 0;
        }

        lastPower = power;

        angleRight.setPower(power); // Set power for right angle motor
        angleLeft.setPower(power); // Set power for left angle motor


        // Debug logging for angle power
        DebugUtils.logDebug(opMode.telemetry, IS_DEBUG_MODE, SUBSYSTEM_NAME, "Angle Power", power);
    }

    // Set power to the angle motors with added force compensation
    public void setPowerAngleWithF(double power) {
        setAnglePower(power + calculateF()); // Apply force compensation to the power
    }

    // Set power to the extension motors
    public void setPowerExtend(double power) {
        // Prevent movement if within power limits
        if (getExtend() >= MAX_EXTEND && power > 0) {
            power = 0;
        }
        extendLeft.setPower(power); // Set power for left extension motor
        extendRight.setPower(power); // Set power for right extension motor

        // Debug logging for extension power
        DebugUtils.logDebug(opMode.telemetry, IS_DEBUG_MODE, SUBSYSTEM_NAME, "Extend Power", power);
    }

    public void setPowerWithLimit(double power){
        if (getAngle() < 50 && getExtend() >= ( 106 - 45 - 20) && power > 0){
            power = 0;
        }
        setPowerExtend(power);
    }


    // Action to move the arm angle
    public Action moveAngle() {
        MoveAngle moveAngle = new MoveAngle(); // Create move angle action
        return moveAngle;
    }

    // Action to check if the angle has reached the setpoint
    public Action setAngle(double goal) {
        SetAngle setAngle = new SetAngle(goal); // Create at setpoint action
        return setAngle;
    }

    // Action to move the arm extension
    public Action moveExtend() {
        MoveExtend moveExtend = new MoveExtend(); // Create move extend action
        return moveExtend;
    }

    public Action setExtend(double goal){
        SetExtend setExtend = new SetExtend(goal);
        return setExtend;
    }

    public Action intaking(double extend){
        return new ParallelAction(setExtend(extend),setAngle(-10));
    }

    // Action class to move the arm angle
    public class MoveAngle implements Action {

        private double PIDPower = 0;

        public MoveAngle() {
            anglePID.setSetPoint(getAngle());
        }


        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
//            if(IS_DEBUG_MODE){
//                anglePID.setP(KP);
//                anglePID.setI(KI);
//                anglePID.setD(KD);
//            }
            PIDPower = anglePID.calculate(getAngle());
            setPowerAngleWithF(PIDPower);
            return true;
        }
    }

    // Action class to check if the angle is at setpoint
    public class SetAngle implements Action {
        private double goal = getAngle(); // Goal for the angle

        private boolean doOne = false;

        public SetAngle(double goal) {
            if (goal > 90){
                this.goal = 90;
            }
            else{
                this.goal = goal;
            }
            doOne = true;
//            anglePID.setTimeout(100000000);
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (doOne){
                anglePID.reset();
                anglePID.setSetPoint(goal);
                doOne = false;
            }
            DebugUtils.logDebug(opMode.telemetry, IS_DEBUG_MODE, SUBSYSTEM_NAME, "setPointAngle", anglePID.getSetPoint());
            // Return true if angle is not yet at the setpoint
            return !anglePID.atSetPoint();
        }
    }

    // Action class to move the arm extension
    public class MoveExtend implements Action {

        private double PIDPower = 0;

        public MoveExtend(){
            extendPID.setSetPoint(getExtend());
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
//                        if(IS_DEBUG_MODE){
//                extendPID.setP(KP);
//                extendPID.setI(KI);
//                extendPID.setD(KD);
//            }
            // Calculate power using PID and apply with limits
//            if (Math.abs(Math.cos(Math.toRadians(getAngle())) * extendPID.getSetPoint()) >= LIMIT){
//                extendPID.setSetPoint(LIMIT);
//            }
            PIDPower = extendPID.calculate(getExtend());

            setPowerExtend(PIDPower);
            return true; // Return true if extension is not at setpoint
        }
    }


    public class SetExtend implements Action{

        private boolean doOne = false;

        private double goal = getAngle();

        public SetExtend(double goal){
            if (goal > getMaxExtend()){
                this.goal = getMaxExtend();
            }
            else if (goal < 0){
                goal = 0;
            }
            else{
                this.goal = goal;
            }
            doOne = true;
//            if (IS_DEBUG_MODE) {
//                extendPID.setTimeout(10000);
//            }
//            goal = MathUtil.wrap(goal, 0, getMaxExtend());

        }

        public boolean run(@NonNull TelemetryPacket telemetryPacket){
            if (doOne){
                extendPID.reset();
                extendPID.setSetPoint(goal);
                doOne = false;
            }
            DebugUtils.logDebug(opMode.telemetry, IS_DEBUG_MODE, SUBSYSTEM_NAME, "getSetPointExtend", extendPID.getSetPoint());
            opMode.telemetry.update();
            return !extendPID.atSetPoint();
        }
    }
}
