package org.firstinspires.ftc.teamcode.tatooine.SubSystem;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.tatooine.utils.DebugUtils;
import org.firstinspires.ftc.teamcode.tatooine.utils.PIDFController;
import org.firstinspires.ftc.teamcode.tatooine.utils.mathUtil.MathUtil;

@Config
public class Wrist {
    // ---------------------------------------------------------------------------------------------
    // Constants
    // ---------------------------------------------------------------------------------------------
    private static final String SUBSYSTEM_NAME = "Wrist";
    public static double HOME = 0.6 ;

    public static double STRIGHT = 0.65;

    public static double SCORESAMPLE = 0.925 ;
    public static double OPEN_MIN = 0.4;
    public static double BACK  = 0.2;


    private final double FRONT = 0.925;
    private static final double ANGLE_TOLERANCE = 10;
    public static final double FULL_RANGE = 291 - 24; // TODO: Verify real values from CAD

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
    private final Servo angleServo;
    private final AnalogInput angleSensor;

    // PID controller
    private final PIDFController pid = new PIDFController(0.005, 0, 0, 0);

    public static double KP = 0.005;

    public static double KI = 0;

    public static double KD = 0;

    public static double speed = 0.1;


    // Telemetry
    private final Telemetry telemetry;

    private double angle = 0;
    private double prevAbsAngle = 0;

    public static double OFFSET = 0;
    public static double WRIST_OFFSET = 0;
    // ---------------------------------------------------------------------------------------------
    // Constructors
    // ---------------------------------------------------------------------------------------------
    public Wrist(OpMode opMode, boolean isDebugMode) {
        this.wristLeft  = opMode.hardwareMap.get(Servo.class, "WAL");
        this.wristRight = opMode.hardwareMap.get(Servo.class, "WAR");
        this.angleServo = opMode.hardwareMap.get(Servo.class, "WA");
        this.angleSensor= opMode.hardwareMap.get(AnalogInput.class, "WAS");
        this.telemetry  = opMode.telemetry;
        this.isDebugMode= isDebugMode;
        init();

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
        wristLeft.setDirection(Servo.Direction.FORWARD);
        wristRight.setDirection(Servo.Direction.FORWARD);
        pid.setTolerance(ANGLE_TOLERANCE);
        angleServo.setDirection(Servo.Direction.REVERSE);

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

    public void scoreSample(){
        setPosition(SCORESAMPLE);
        currentPos = SCORESAMPLE;
        DebugUtils.logDebug(telemetry, isDebugMode, SUBSYSTEM_NAME,
                "Wrist ScoreSample", SCORESAMPLE);
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

    public void home(){
        setPosition(HOME);
        currentPos = HOME;
        DebugUtils.logDebug(telemetry, isDebugMode, SUBSYSTEM_NAME,
                "Wrist HOME", HOME);
    }

    public void stright(){
        setPosition(STRIGHT);
        currentPos = STRIGHT;
        DebugUtils.logDebug(telemetry, isDebugMode, SUBSYSTEM_NAME,
                "Wrist STRIGHT", STRIGHT);
    }

    public void openMin(){
        setPosition(OPEN_MIN);
        currentPos = OPEN_MIN;
        DebugUtils.logDebug(telemetry, isDebugMode, SUBSYSTEM_NAME,
                "Wrist OPEN_MIN", OPEN_MIN);
    }



    // ---------------------------------------------------------------------------------------------
    // Angle Servo Control
    // ---------------------------------------------------------------------------------------------
    public void setPosAng(double pos) {
        angleServo.setPosition(pos- WRIST_OFFSET);
        DebugUtils.logDebug(telemetry, isDebugMode, SUBSYSTEM_NAME,
                "Servo pos Set", pos);
    }

    public double getAngle() {
        double angle = -MathUtil.normalizeAngleTo180(MathUtil.voltageToDegrees(angleSensor.getVoltage())- OFFSET);
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

    public double getState(){
        return currentPos;
    }

    // ---------------------------------------------------------------------------------------------
    // Inner Classes (Actions)
    // ---------------------------------------------------------------------------------------------
    public class MoveAngle implements Action {
        private double goal;

        public MoveAngle(double goal) {
//            if (goal <= 90){this.goal = (goal/180)-0.5;}
//            else {this.goal = (goal/180)+0.5;}
            this.goal = goal/180 -OFFSET;
            DebugUtils.logDebug(telemetry, isDebugMode, SUBSYSTEM_NAME,
                    "MoveAngle Initialized", this.goal);
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            angle = getAngle();
            angleServo.setPosition(goal);
            DebugUtils.logDebug(telemetry, isDebugMode, SUBSYSTEM_NAME, "servoGetPos",angleServo.getPosition());
            DebugUtils.logDebug(telemetry, isDebugMode, SUBSYSTEM_NAME, "goal", goal);
            return !MathUtil.inTolerance(angle, 180 * goal, ANGLE_TOLERANCE );
        }
    }
}
