package org.firstinspires.ftc.teamcode.bots;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.CVRec.CVDetectMode;
import org.firstinspires.ftc.teamcode.CVRec.CVDetector;
import org.firstinspires.ftc.teamcode.CVRec.GameElement;
import org.firstinspires.ftc.teamcode.autonomous.AutoDot;
import org.firstinspires.ftc.teamcode.autonomous.AutoRoute;

public class FrenzyBotI extends FrenzyBaseBotI {
    private DcMotorEx intake = null;
    private DcMotorEx lift = null;
    private DcMotorEx rotator = null;
    private Servo dropperServo = null;
    private static final String TAG = "FrenzyBot";
    private static int LIFT_FULL_EXTENSION = -1380;
    private static int LIFT_HALF_EXTENSION = -650;
    private static int LIFT_NO_EXTENSION = 0;
    private static double LIFT_SPEED = 0.8;

    // Dropper Servo positions
    private static double DROPPER_SERVO_POS_READY = 0.5;
    private static double DROPPER_SERVO_POS_DROP = 0.0;

    // Detection
    CVDetector detector;
    String opModeSide = AutoRoute.NAME_RED;

    private GameElement detectedElement;

    /* Constructor */
    public FrenzyBotI() {
        opModeSide = AutoRoute.NAME_RED; // defult
    }

    public FrenzyBotI(String fieldSide) {
        this.opModeSide = fieldSide;
    }

    @Override
    public void init(LinearOpMode owner, HardwareMap ahwMap, Telemetry telemetry) throws Exception {
        super.init(owner, ahwMap, telemetry);
        try {
            intake = hwMap.get(DcMotorEx.class, "intake");
            intake.setDirection(DcMotor.Direction.FORWARD);
            intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            intake.setVelocity(0);
        } catch (Exception ex) {
            Log.e(TAG, "Cannot initialize intake", ex);
        }
        try {
            lift = hwMap.get(DcMotorEx.class, "lift");
            lift.setDirection(DcMotor.Direction.REVERSE);
            lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            lift.setVelocity(0);
        } catch (Exception ex) {
            Log.e(TAG, "Cannot initialize lift", ex);
        }
        try {
            rotator = hwMap.get(DcMotorEx.class, "rotator");
            rotator.setDirection(DcMotor.Direction.FORWARD);
            rotator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rotator.setVelocity(0);
        } catch (Exception ex) {
            Log.e(TAG, "Cannot initialize rotator", ex);
        }
        try {
            dropperServo =  hwMap.get(Servo.class, "dropper");
            dropperServo.setPosition(DROPPER_SERVO_POS_READY);
        } catch (Exception ex) {
            Log.e(TAG, "Cannot initialize dropperServo", ex);
        }

    }

    @Override
    public void initDetectorThread(String side, LinearOpMode caller) {
        try {
            detector = new CVDetector(hwMap, opModeSide, CVDetectMode.Frenzy, this.namedCoordinates);
            detector.startDetection();
            telemetry.addData("Info", "Detector initialized");
            telemetry.update();
        } catch (Exception ex) {
            Log.e(TAG, "Cannot initialize Detector", ex);
        }
    }

    @BotAction(displayName = "Get Detection Result", defaultReturn = "C")
    @Override
    public AutoDot getDetectionResult() {
        AutoDot level = detector.getLevel();
        Log.d(TAG, String.format("Detection result: Level $s", level.getDotName()));
        telemetry.addData("Level: ", level);
        telemetry.update();
        detector.stopDetection();

        return level;
    }

    public void activateIntake(double velocity) {
        if (intake != null) {
            intake.setVelocity(MAX_VELOCITY_REV*velocity);
        }
    }
    public void activateLift(double velocity) {
        if (lift != null) {
            lift.setVelocity(MAX_VELOCITY_REV*velocity);
        }
    }
    public void activateRotator(double velocity) {
        if (rotator != null) {
            rotator.setVelocity(MAX_VELOCITY_REV*velocity);
        }
    }

    public int getLiftPosition(){
        return this.lift.getCurrentPosition();
    }

    @BotAction(displayName = "Lift to upper", defaultReturn = "")
    public void liftToUpper(){
        this.lift.setTargetPosition(LIFT_FULL_EXTENSION);
        this.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.lift.setVelocity(MAX_VELOCITY_REV*LIFT_SPEED);
    }

    @BotAction(displayName = "Lift to mid", defaultReturn = "")
    public void liftToMid(){
        this.lift.setTargetPosition(LIFT_HALF_EXTENSION);
        this.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.lift.setVelocity(MAX_VELOCITY_REV*LIFT_SPEED);
    }

    @BotAction(displayName = "Lift to lower", defaultReturn = "")
    public void liftToLower(){
        this.lift.setTargetPosition(LIFT_NO_EXTENSION);
        this.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.lift.setVelocity(MAX_VELOCITY_REV*LIFT_SPEED);
    }

    @BotAction(displayName = "Drop element", defaultReturn = "")
    public void dropElement(){
        if (dropperServo != null) {
            dropperServo.setPosition(DROPPER_SERVO_POS_DROP);
        }
    }

    @BotAction(displayName = "Reset dropper", defaultReturn = "")
    public void resetDropper(){
        if (dropperServo != null) {
            dropperServo.setPosition(DROPPER_SERVO_POS_READY);
        }
    }

    @BotAction(displayName = "Start intake", defaultReturn = "")
    public void startIntake() {
        activateIntake(0.95);
    }

    @BotAction(displayName = "Reverse intake", defaultReturn = "")
    public void reverseIntake() {
        activateIntake(-0.75);
    }

    @BotAction(displayName = "Stop intake", defaultReturn = "")
    public void stopIntake() {
        activateIntake(0);
    }

    public void startTurntableBlue() {
        activateRotator(0.75);
    }
    public void startTurntableRed() {
        activateRotator(-0.75);
    }
    public void stopTurntable() {
        activateRotator(0.0);
    }
}
