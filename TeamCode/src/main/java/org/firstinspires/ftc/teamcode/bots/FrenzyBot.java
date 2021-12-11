package org.firstinspires.ftc.teamcode.bots;

import android.graphics.Color;
import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import com.qualcomm.robotcore.util.ElapsedTime;

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
    public static int LIFT_LEVEL_THREE = -2000;
    public static int LIFT_LEVEL_TWO = -1720;
    public static int LIFT_LEVEL_ONE = -860; // Make sure to be further away for level one
    public static int LIFT_NO_EXTENSION = 0;
    public static int LIFT_UNDER_EXTENTION = 5;

    private int liftLocation = LIFT_NO_EXTENSION;
    private static double LIFT_SPEED = 0.95;
    private static double LIFT_SPEED_LOW = 0.7;

    NormalizedColorSensor colorSensor;

    // Dropper Servo positions
    private static double DROPPER_SERVO_POS_PICKUP = 0.97; // this is only to pick-up elements
    private static double DROPPER_SERVO_POS_START = 0.97;  //default pos to start and transport
    private static double DROPPER_SERVO_POS_DROP = 0.0;

    // Detection
    CVDetector detector;
    String opModeSide = AutoRoute.NAME_RED;
    private GameElement detectedElement;


    //Intake
    private static double INTAKE_ELEMENT_MOVE_SPEED = -0.2;
    private static double INTAKE_SPEED = 0.7;
    private static double INTAKE_SPEED_REVERSE = -0.65;

    private boolean intakeRunning = false;


    /* Constructor */
    public FrenzyBot() {
        opModeSide = AutoRoute.NAME_RED; // default
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
            prepDropperToMove();
        } catch (Exception ex) {
            Log.e(TAG, "Cannot initialize dropperServo", ex);
        }
        try {
            colorSensor = hwMap.get(NormalizedColorSensor.class, "colorSensor");
        } catch (Exception ex) {
            Log.e(TAG, "Cannot initialize colorSensor", ex);
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
        Log.d(TAG, String.format("Detection result: Level %s", level.getDotName()));
        telemetry.addData("Level: ", level);
        telemetry.update();
        detector.stopDetection();

        return level;
    }

    ///For status only
    @Override
    public AutoDot getCurrentDetectionResult() {
        AutoDot level = detector.getLevel();
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
        //reset dropper before retracting the lift all the way
        if (dropperServo.getPosition() < 0.5) {
            resetDropper();
            delayWait(999);
        }else {
            resetDropper();
        }
        liftLocation = LIFT_UNDER_EXTENTION;
        this.lift.setTargetPosition(LIFT_UNDER_EXTENTION);
        this.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        if (getLiftPosition() == LIFT_LEVEL_ONE){
            this.lift.setVelocity(MAX_VELOCITY_REV * LIFT_SPEED_LOW);
        }
        else {
            this.lift.setVelocity(MAX_VELOCITY_REV * LIFT_SPEED);
        }
    }

    public boolean isLiftBusy(){
        return this.lift.isBusy();
    }

    private void stopLift(){
        this.lift.setPower(0);
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
        prepDropperToMove();
    }

    @BotAction(displayName = "Prep dropper", defaultReturn = "")
    public void prepDropperToMove(){
        if (dropperServo != null) {
            dropperServo.setPosition(DROPPER_SERVO_POS_START);
        }
    }

    public void dropperPickupPosition(){
        if (dropperServo != null) {
            dropperServo.setPosition(DROPPER_SERVO_POS_PICKUP);
        }
    }

    @BotAction(displayName = "Start intake", defaultReturn = "")
    public void startIntake() {
        activateIntake(INTAKE_SPEED);
        intakeRunning = true;
        dropperPickupPosition();
    }

    @BotAction(displayName = "Reverse intake", defaultReturn = "")
    public void reverseIntake() {
        activateIntake(INTAKE_SPEED_REVERSE);
    }

    @BotAction(displayName = "Stop intake", defaultReturn = "")
    public void stopIntake() {
        activateIntake(0);
        intakeRunning = false;
        prepDropperToMove();
    }

    @BotAction(displayName = "Start turntable blue", defaultReturn = "")
    public void startTurntableBlue() {
        activateRotatorRight(0.35);
        activateRotatorLeft(0.35);
    }
    @BotAction(displayName = "Start turntable red", defaultReturn = "")
    public void startTurntableRed() {
        activateRotatorRight(-0.35);
        activateRotatorLeft(-0.35);
    }

    protected void delayWait(long ms) {
        ElapsedTime time = new ElapsedTime();
        time.reset();
        while (time.milliseconds() < ms){
            // do nothing
        }
    }

    @BotAction(displayName = "Start turntable blue gradual", defaultReturn = "")
    public void startTurntableBlueGradual() {
        duckLoop(false, false);
    }
    @BotAction(displayName = "Start turntable red gradual", defaultReturn = "")
    public void startTurntableRedGradual() {
        duckLoop(true, false);
    }

    @BotAction(displayName = "Start turntable blue auto gradual", defaultReturn = "")
    public void startTurntableBlueAutoGradual() {
        duckLoop(false, true);
    }
    @BotAction(displayName = "Start turntable red auto gradual", defaultReturn = "")
    public void startTurntableRedAutoGradual() {
        duckLoop(true, true);
    }

    @BotAction(displayName = "Stop turntable", defaultReturn = "")
    public void stopTurntable() {
        activateRotatorRight(0.0);
        activateRotatorLeft(0.0);
    }
    public void toggleLight(boolean on){
        if (colorSensor != null && colorSensor instanceof SwitchableLight) {
            ((SwitchableLight) colorSensor).enableLight(on);
        }
    }

    public float detectColor(Telemetry telemetry, float timeout) {
        ElapsedTime runtime = new ElapsedTime();
        toggleLight(true);

        float[] hsvValues = new float[3];

        boolean stop = false;
        while(!stop) {
            NormalizedRGBA colors = colorSensor.getNormalizedColors();
            Color.colorToHSV(colors.toColor(), hsvValues);
            stop = timeout == 0 || (timeout > 0 && runtime.seconds() >= timeout);
        }

        toggleLight(false);
        //returning H value
        return hsvValues[0];
    }

    public boolean isIntakeBoxEmpty(){
        float HValue = detectColor(telemetry, 0);
        return HValue < 5;
    }

    @BotAction(displayName = "Grab Element", defaultReturn = "")
    public Boolean grabElement(){
        Boolean gotIt = false;
        ElapsedTime runtime = new ElapsedTime();
        runtime.reset();
        startIntake();
        //move straight until an element is detected in the intake box
        moveAuto(INTAKE_ELEMENT_MOVE_SPEED);
        while (owner.opModeIsActive()){
            if (!isIntakeBoxEmpty()){
                Log.d(TAG, "Element is in the intake");
                gotIt = true;
                break;
            }
            if (runtime.milliseconds() > 4000){
                Log.d(TAG, "Ran out of time");
                break;
            }
        }

        stopIntake();
        stop();

        return gotIt;
    }

    public void goDucks(boolean red){
        for(int i = 0; i < 12; i++){
            duckLoop(red, false);
            delayWait(600);
        }
    }

    private void duckLoop(boolean red, boolean auto){
        double startSpeed = -0.2;
        double speedIncrement = -0.01;
        int maxLoops = 10;
        int loopDelayMs = 140;
        double maxSpeed = 0.25;
        if (auto){
            maxSpeed = 0.055;
            maxLoops = maxLoops*2;
        }
        if (red){
            maxSpeed = - maxSpeed;
        }
        double currSpeed = startSpeed;
        for(int i = 0; i < maxLoops; i++){
            currSpeed += speedIncrement;
            if (currSpeed < maxSpeed) {
                currSpeed = maxSpeed;
            }
            activateRotatorRight(currSpeed);
            activateRotatorLeft(currSpeed);

            this.delayWait(loopDelayMs);
        }
        activateRotatorRight(0.0);
        activateRotatorLeft(0.0);
    }

    public boolean isIntakeRunning() {
        return intakeRunning;
    }
}
