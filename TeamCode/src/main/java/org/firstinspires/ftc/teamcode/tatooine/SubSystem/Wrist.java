package org.firstinspires.ftc.teamcode.tatooine.SubSystem;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.tatooine.utils.DebugUtils;
import org.firstinspires.ftc.teamcode.tatooine.utils.PIDFController;
import org.firstinspires.ftc.teamcode.tatooine.utils.mathUtil.MathUtil;

public class Wrist {
    // ---------------------------------------------------------------------------------------------
    // Constants
    // ---------------------------------------------------------------------------------------------
    private static final String SUBSYSTEM_NAME = "Wrist";
    private static final double FRONT = 1;
    private static final double BACK  = 0;
    private static final double ANGLE_TOLERANCE = 3;
    private static final double FULL_RANGE = 291 - 24; // TODO: Verify real values from CAD

    // ---------------------------------------------------------------------------------------------
    // State Variables
    // ---------------------------------------------------------------------------------------------
    private double currentPos = 0;
    private boolean isDebugMode;
    private boolean shouldStayParallel = false;

    // ---------------------------------------------------------------------------------------------
    // Hardware Components
    // ---------------------------------------------------------------------------------------------
    private final Servo wristLeft;
    private final Servo wristRight;
    private final CRServo angleServo;
    private final AnalogInput angleSensor;

    // PID controller
    private final PIDFController pid = new PIDFController(0.0025, 0.05, 0.001, 0);

    // Telemetry
    private final Telemetry telemetry;

    // ---------------------------------------------------------------------------------------------
    // Constructors
    // ---------------------------------------------------------------------------------------------
    public Wrist(OpMode opMode, boolean isDebugMode) {
        this.wristLeft  = opMode.hardwareMap.get(Servo.class, "WAL");
        this.wristRight = opMode.hardwareMap.get(Servo.class, "WAR");
        this.angleServo = opMode.hardwareMap.get(CRServo.class, "WA");
        this.angleSensor= opMode.hardwareMap.get(AnalogInput.class, "WAS");
        this.telemetry  = opMode.telemetry;
        this.isDebugMode= isDebugMode;

        DebugUtils.logDebug(telemetry, isDebugMode, SUBSYSTEM_NAME,
                "Constructor initialized", "Success");
    }

    public Wrist(OpMode opMode) {
        this(opMode, false);
    }

    // ---------------------------------------------------------------------------------------------
    // Initialization
    // ---------------------------------------------------------------------------------------------
    public void init() {
        wristLeft.setDirection(Servo.Direction.REVERSE);
        wristRight.setDirection(Servo.Direction.FORWARD);
        pid.setTolerance(ANGLE_TOLERANCE);
        angleServo.setPower(pid.calculate(getAngle(), 0));

        DebugUtils.logDebug(telemetry, isDebugMode, SUBSYSTEM_NAME,
                "Initialization", "Completed");
    }

    // ---------------------------------------------------------------------------------------------
    // Debug Mode
    // ---------------------------------------------------------------------------------------------
    public boolean isDebugMode() {
        return isDebugMode;
    }

    // ---------------------------------------------------------------------------------------------
    // Wrist State Control
    // ---------------------------------------------------------------------------------------------
    public void changeState() {
        if (currentPos == FRONT) {
            close();
        } else {
            open();
        }
        DebugUtils.logDebug(telemetry, isDebugMode, SUBSYSTEM_NAME,
                "State Changed", currentPos);
    }

    public void open() {
        setPosition(FRONT);
        currentPos = FRONT;
        DebugUtils.logDebug(telemetry, isDebugMode, SUBSYSTEM_NAME,
                "Wrist Opened", FRONT);
    }

    public void close() {
        setPosition(BACK);
        currentPos = BACK;
        DebugUtils.logDebug(telemetry, isDebugMode, SUBSYSTEM_NAME,
                "Wrist Closed", BACK);
    }

    // ---------------------------------------------------------------------------------------------
    // Angle Servo Control
    // ---------------------------------------------------------------------------------------------
    public void setPower(double power) {
        angleServo.setPower(power);
        DebugUtils.logDebug(telemetry, isDebugMode, SUBSYSTEM_NAME,
                "Servo Power Set", power);
    }

    public double getAngle() {
        double angle = MathUtil.voltageToDegrees(angleSensor.getVoltage());
        DebugUtils.logDebug(telemetry, isDebugMode, SUBSYSTEM_NAME,
                "Angle Read", angle);
        return angle;
    }

    // ---------------------------------------------------------------------------------------------
    // Position Control
    // ---------------------------------------------------------------------------------------------
    public void setPosition(double position) {
        wristLeft.setPosition(position);
        wristRight.setPosition(position);
        DebugUtils.logDebug(telemetry, isDebugMode, SUBSYSTEM_NAME,
                "Position Set", position);
    }

    public double getCurrentPos() {
        DebugUtils.logDebug(telemetry, isDebugMode, SUBSYSTEM_NAME,
                "Get Current Position", currentPos);
        return currentPos;
    }

    public void setCurrentPos(double currentPos) {
        this.currentPos = currentPos;
        DebugUtils.logDebug(telemetry, isDebugMode, SUBSYSTEM_NAME,
                "Set Current Position", currentPos);
    }

    // ---------------------------------------------------------------------------------------------
    // Parallel / Specimen Actions
    // ---------------------------------------------------------------------------------------------
    /**
     * Moves the wrist to a specific angle using the internal PID controller.
     * @param angle the target angle in degrees
     * @return an Action that will run until the wrist is at setpoint
     */
    public Action moveToAngle(double angle) {
        return new MoveAngle(angle);
    }

    /**
     * Example 'specimen' action that sets the wrist to some angle suitable for scoring a specimen.
     * Adjust the angle as needed.
     */
    public Action specimen() {
        // For demonstration, let's assume 45 degrees is our 'specimen' angle
        return moveToAngle(45);
    }

    /**
     * Keeps the wrist parallel to the floor, referencing the 'armAngle' inside the Action itself.
     * Useful if the arm moves and you want the wrist to counteract that motion.
     */
    public Action parallelToFloor() {
        return new ParallelToFloor();
    }

    public void setShouldStayParallel(boolean shouldStayParallel) {
        this.shouldStayParallel = shouldStayParallel;
        DebugUtils.logDebug(telemetry, isDebugMode, SUBSYSTEM_NAME,
                "Set Should Stay Parallel", shouldStayParallel);
    }

    public boolean getShouldStayParallel() {
        DebugUtils.logDebug(telemetry, isDebugMode, SUBSYSTEM_NAME,
                "Get Should Stay Parallel", shouldStayParallel);
        return shouldStayParallel;
    }

    // ---------------------------------------------------------------------------------------------
    // Inner Classes (Actions)
    // ---------------------------------------------------------------------------------------------
    public class MoveAngle implements Action {
        private final double goal;

        public MoveAngle(double goal) {
            pid.reset();
            this.goal = goal;
            DebugUtils.logDebug(telemetry, isDebugMode, SUBSYSTEM_NAME,
                    "MoveAngle Initialized", goal);
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            double currentAngle = getAngle();
            DebugUtils.logDebug(telemetry, isDebugMode, SUBSYSTEM_NAME,
                    "MoveAngle Running", currentAngle);

            pid.setTimeout(100000000);
            angleServo.setPower(-pid.calculate(currentAngle, goal));

            if (pid.atSetPoint()) {
                angleServo.setPower(0);
                DebugUtils.logDebug(telemetry, isDebugMode, SUBSYSTEM_NAME,
                        "MoveAngle Completed", currentAngle);
            }
            return !pid.atSetPoint();
        }
    }

    public class ParallelToFloor implements Action {
        private double armAngle = 0;

        public ParallelToFloor() {
            DebugUtils.logDebug(telemetry, isDebugMode, SUBSYSTEM_NAME,
                    "ParallelToFloor Initialized", "Success");
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            double offset = Math.cos(Math.toRadians(armAngle)) * 90 - FRONT;
            offset /= FULL_RANGE;
            if (armAngle < 0) {
                offset += 90 / FULL_RANGE;
            }

            offset = Math.min(Math.max(offset, BACK), FRONT);
            setPosition(offset);

            DebugUtils.logDebug(telemetry, isDebugMode, SUBSYSTEM_NAME,
                    "ParallelToFloor Offset", offset);
            return shouldStayParallel;
        }
    }
}
