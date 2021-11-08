package org.firstinspires.ftc.teamcode.bots;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.CVRec.CVDetectMode;
import org.firstinspires.ftc.teamcode.CVRec.CVDetector;
import org.firstinspires.ftc.teamcode.CVRec.GameElement;
import org.firstinspires.ftc.teamcode.autonomous.AutoDot;
import org.firstinspires.ftc.teamcode.autonomous.AutoRoute;

public class FrenzyBot extends FrenzyBaseBot {
    private DcMotorEx intake = null;
    private DcMotorEx lift = null;
    private DcMotorEx rotatorRight = null;
    private DcMotorEx rotatorLeft = null;
    private Servo dropperServo = null;
    private static final String TAG = "FrenzyBot";
    public static int LIFT_LEVEL_THREE = -3450; // TODO: 11/7/21 Change values based on the empirical observations 
    public static int LIFT_LEVEL_TWO = -1840;
    public static int LIFT_LEVEL_ONE = -740;
    public static int LIFT_NO_EXTENSION = 0;

    private int liftLocation = LIFT_NO_EXTENSION;
    private static double LIFT_SPEED = 0.8;

    // Dropper Servo positions
    private static double DROPPER_SERVO_POS_READY = 0.5;
    private static double DROPPER_SERVO_POS_DROP = 0.0;

    // Detection
    CVDetector detector;
    String opModeSide = AutoRoute.NAME_RED;

    private GameElement detectedElement;

    /* Constructor */
    public FrenzyBot() {
        opModeSide = AutoRoute.NAME_RED; // defult
    }

    public FrenzyBot(String fieldSide) {
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
            lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        } catch (Exception ex) {
            Log.e(TAG, "Cannot initialize lift", ex);
        }
        try {
            rotatorRight = hwMap.get(DcMotorEx.class, "rotatorRight");
            rotatorRight.setDirection(DcMotor.Direction.FORWARD);
            rotatorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rotatorRight.setVelocity(0);
        } catch (Exception ex) {
            Log.e(TAG, "Cannot initialize rotator", ex);
        }
        try {
            rotatorLeft = hwMap.get(DcMotorEx.class, "rotatorLeft");
            rotatorLeft.setDirection(DcMotor.Direction.FORWARD);
            rotatorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rotatorLeft.setVelocity(0);
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
    public void activateRotatorRight(double velocity) {
        if (rotatorRight != null) {
            rotatorRight.setVelocity(MAX_VELOCITY_REV*velocity);
        }
    }
    public void activateRotatorLeft(double velocity) {
        if (rotatorLeft != null) {
            rotatorLeft.setVelocity(MAX_VELOCITY_REV*velocity);
        }
    }

    public int getLiftPosition(){

        return this.lift.getCurrentPosition();
    }

    @BotAction(displayName = "Lift level 3", defaultReturn = "")
    public void liftToLevel3(){
        liftLocation = LIFT_LEVEL_THREE;
        this.lift.setTargetPosition(LIFT_LEVEL_THREE);
        this.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.lift.setVelocity(MAX_VELOCITY_REV*LIFT_SPEED);
    }

    @BotAction(displayName = "Lift level 2", defaultReturn = "")
    public void liftToLevel2(){
        liftLocation = LIFT_LEVEL_TWO;
        this.lift.setTargetPosition(LIFT_LEVEL_TWO);
        this.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.lift.setVelocity(MAX_VELOCITY_REV*LIFT_SPEED);
    }

    @BotAction(displayName = "Lift level 1", defaultReturn = "")
    public void liftToLevel1(){
        liftLocation = LIFT_LEVEL_ONE;
        this.lift.setTargetPosition(LIFT_LEVEL_ONE);
        this.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.lift.setVelocity(MAX_VELOCITY_REV*LIFT_SPEED);
    }

    @BotAction(displayName = "Lift to lower", defaultReturn = "")
    public void liftToLower(){
        liftLocation = LIFT_NO_EXTENSION;
        this.lift.setTargetPosition(LIFT_NO_EXTENSION);
        this.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.lift.setVelocity(MAX_VELOCITY_REV*LIFT_SPEED);
    }

    public int getLiftLocation(){
        return liftLocation;
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

    @BotAction(displayName = "Start turntable blue", defaultReturn = "")
    public void startTurntableBlue() {
        activateRotatorRight(0.75);
        activateRotatorLeft(0.75);
    }
    @BotAction(displayName = "Start turntable red", defaultReturn = "")
    public void startTurntableRed() {
        activateRotatorRight(-0.75);
        activateRotatorLeft(-0.75);
    }
    @BotAction(displayName = "Stop turntable", defaultReturn = "")
    public void stopTurntable() {
        activateRotatorRight(0.0);
        activateRotatorLeft(0.0);
    }
}
