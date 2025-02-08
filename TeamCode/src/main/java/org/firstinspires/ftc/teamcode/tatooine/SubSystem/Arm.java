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

    private double startAngle = -5; // Initial starting angle of the arm

    private double ANGLE_OFFSET = -5;   // Angle offset for the arm

    private DigitalChannel limitSwitch = null; // Limit switch to detect arm position limits

    private final double ANGLE_AMP_LIMIT = 7000.0; // Maximum allowed amperage for angle motors


    // Extend system variables
    private DcMotorEx extendLeft = null; // Left motor for extension
    private DcMotorEx extendRight = null; // Right motor for extension

    private final double EXTEND_UP_LIMIT_AMPS = 0; // Upper amperage limit for extend motors
    private final double EXTEND_DOWN_LIMIT_AMPS = 5000.0; // Lower amperage limit for extend motors
    public static double MIN_EXTEND = 37.0; // Minimum extension value
    public static double MAX_EXTEND = 67.0; // Maximum extension value

    public static double LIMIT = UnitConverter.convert(42.0, unit.INCHES, unit.CM) - 20.0 - 5.0; // Conversion for limit distance

    private PIDFController anglePID = new PIDFController(0.1, 0.0, 0.005, 0); // PID controller for angle movement
    private PIDFController extendPID = new PIDFController(0.2, 0.00001, 0.0, 0); // PID controller for extension movement

    private final double ANGLE_CPR = 28.0 * 100.0 * (34.0 / 16.0); // Counts per revolution for angle motors
    private final double EXTEND_CPR = 537.7; // Counts per revolution for extension motors

    private final double ANGLE_TIMEOUT = 3.0; // Timeout for angle movement
    private final double EXTEND_TIMEOUT = 4.5; // Timeout for extension movement

    private final double ANGLE_TOLERANCE = 0.5; // Tolerance for angle movement
    private final double EXTEND_TOLERANCE = 1.0; // Tolerance for extension movement

    private final double LIMIT_TOLERANCE = 0.0; // Tolerance for limit switch reading

    private final double SPOOL_DIM = 4.0; // Spool dimensions for extension calculation

    private double lastAngle = 0.0; // Last recorded angle for force calculation
    private double lastF = 0.0; // Last recorded force value

    private String SUBSYSTEM_NAME = "Arm"; // Name of the subsystem

    private boolean IS_DEBUG_MODE = false; // Flag to indicate if debug mode is enabled

    private OpMode opMode = null; // OpMode reference

    public static double KF = 0.15; // Constant factor for force calculation

    public static double KP = 0;

    public static double KI = 0;

    public static double KD = 0;

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
        return MathUtil.convertTicksToDegrees(ANGLE_CPR, angleRight.getCurrentPosition()) + startAngle;
    }

    // Get the current angle of the right arm motor
    public double getRightAngle() {
        return MathUtil.convertTicksToDegrees(ANGLE_CPR, angleLeft.getCurrentPosition()) + startAngle;
    }

    // Get the average angle of the arm
    public double getAngle() {
        double angle = (getRightAngle() + getLeftAngle()) / 2;
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
        if (getCurrentAngle() >= ANGLE_AMP_LIMIT && power > 0) {
            power = 0; // No movement if amperage limit is exceeded
        }

        if (getAngle() >= 90 && power > 0){
            power = 0;
        }

        // Reset encoders if limit switch is pressed
        if (limitSwitch.getState()) {
            resetAngleEncoders(); // Reset encoders if limit switch is pressed
            startAngle = ANGLE_OFFSET;
            if (power < 0) {
                power = 0; // Prevent negative power if limit switch is pressed
            }
        }

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
        //TODO: rewrite Amper Limit for extention
        if (power < 0 && getCurrentExtend() >= EXTEND_DOWN_LIMIT_AMPS){
            power = 0;
            resetExtendEncoders();
        } else if (getExtend() >= MAX_EXTEND && power > 0) {
            power = 0;
        }
        extendLeft.setPower(power); // Set power for left extension motor
        extendRight.setPower(power); // Set power for right extension motor

        // Debug logging for extension power
        DebugUtils.logDebug(opMode.telemetry, IS_DEBUG_MODE, SUBSYSTEM_NAME, "Extend Power", power);
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
        return new ParallelAction(setExtend(extend), setAngle((extend/ getMaxExtend())));
    }

    // Action class to move the arm angle
    public class MoveAngle implements Action {

        private double PIDPower = 0;

        public MoveAngle() {
            anglePID.setSetPoint(getAngle());
        }


        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            PIDPower = anglePID.calculate(getAngle());
            setPowerAngleWithF(PIDPower);
            return true;
        }
    }

    // Action class to check if the angle is at setpoint
    public class SetAngle implements Action {
        private double goal = getAngle(); // Goal for the angle

        public SetAngle(double goal) {
            this.goal = goal; // Set goal for the angle
            anglePID.reset();
            anglePID.setSetPoint(goal); // Update PID setpoint
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            // Return true if angle is not yet at the setpoint
            DebugUtils.logDebug(opMode.telemetry, IS_DEBUG_MODE, SUBSYSTEM_NAME, "setPointAngle", anglePID.getSetPoint());
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
            if (IS_DEBUG_MODE){
                extendPID.setP(KP);
                extendPID.setI(KI);
                extendPID.setD(KD);
            }
            // Calculate power using PID and apply with limits
            if (Math.abs(Math.cos(Math.toRadians(getAngle())) * getExtend()) >= LIMIT){
                extendPID.setSetPoint(LIMIT);
            }
            PIDPower = extendPID.calculate(getExtend());
            setPowerExtend(PIDPower);

            return true; // Return true if extension is not at setpoint
        }
    }


    public class SetExtend implements Action{

        public SetExtend(double goal){
            extendPID.setTimeout(10000);
            extendPID.reset();
//            goal = MathUtil.wrap(goal, 0, getMaxExtend());
            extendPID.setSetPoint(goal);

        }

        public boolean run(@NonNull TelemetryPacket telemetryPacket){
            DebugUtils.logDebug(opMode.telemetry, IS_DEBUG_MODE, SUBSYSTEM_NAME, "setPointExtend", extendPID.getSetPoint());
            return !extendPID.atSetPoint();
        }
    }
}
