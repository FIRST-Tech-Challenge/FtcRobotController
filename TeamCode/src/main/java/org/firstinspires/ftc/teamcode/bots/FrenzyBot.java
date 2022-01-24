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
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.collections.SimpleGson;
import org.firstinspires.ftc.teamcode.CVRec.CVDetectMode;
import org.firstinspires.ftc.teamcode.CVRec.CVDetector;
import org.firstinspires.ftc.teamcode.CVRec.GameElement;
import org.firstinspires.ftc.teamcode.autonomous.AutoDot;
import org.firstinspires.ftc.teamcode.autonomous.AutoRoute;
import org.firstinspires.ftc.teamcode.calibration.FreightFrenzyConfig;
import org.firstinspires.ftc.teamcode.skills.FrenzyIntake;
import org.firstinspires.ftc.teamcode.skills.FrenzyLift;
import org.firstinspires.ftc.teamcode.skills.FrenzyLiftMode;

public class FrenzyBot extends FrenzyBaseBot {
    private DcMotorEx intake = null;
    private DcMotorEx lift = null;
    private DcMotorEx rotatorLeft = null;
    private DcMotorEx turret = null;
    private Servo dropperServo = null;
    private Servo intakeDropperServo = null;
    private Servo tower = null;

    FreightFrenzyConfig frenzyConfig = null;

    private static final String TAG = "FrenzyBot";
    public static int LIFT_LEVEL_THREE = 1880;
    public static int LIFT_LEVEL_TWO = 1650;
    public static int LIFT_LEVEL_ONE = 1380;
    public static int LIFT_SHARED_HUB = 400;
    public static int LIFT_MIN_EXTENSION = 450;
    public static int LIFT_UNDER_EXTENTION = 0;

    protected static int TURRET_POS_CENTER = 0;
    protected static int TURRET_POS_MAX_LEFT = 840;  //red side team hub
    protected static int TURRET_POS_MAX_RIGHT = -728; //blue side team hub
    protected static int TURRET_POS_TEAMHUB_RED = 610;  //red side team hub
    protected static int TURRET_POS_TEAMHUB_BLUE = -620;  //red side team hub
    protected static int TURRET_POS_SHAREDHUB_RED = -634;  //red side team hub
    protected static int TURRET_POS_SHAREDHUB_BLUE = 700;  //red side team hub
    private static double TURRET_SPEED = 0.95;
    private static double TURRET_SPEED_LOW = 0.8;

    private boolean liftEmergencyMode = false; //if the lift is broken, operate with the intake


    private int liftLocation = LIFT_UNDER_EXTENTION;
    private static double LIFT_SPEED = 0.95;
    private static double LIFT_SPEED_LOW = 0.7;
    protected static int positionToleranceLift = 15;
    protected static int positionToleranceTurret = 5;

    NormalizedColorSensor colorSensor;

    // Dropper Servo positions
    private static double DROPPER_SERVO_POS_TRANSPORT = 0.45; // this is only to pick-up elements
    private static double DROPPER_SERVO_POS_START = 0.3;  //default pos to start and transport
    private static double DROPPER_SERVO_POS_DROP = 1;

    // Detection
    CVDetector detector;
    String opModeSide = AutoRoute.NAME_RED;
    private GameElement detectedElement;


    //Intake
    private static double INTAKE_ELEMENT_MOVE_SPEED = 0.2;
    private static double INTAKE_SPEED = -0.4;
    private static double INTAKE_SPEED_REVERSE = 0.3;

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
            this.frenzyConfig = this.botConfig.getFreightFrenzyConfig();
            Log.d(TAG, "FreightFrenzy BotConfig loaded");
            Log.d(TAG, SimpleGson.getInstance().toJson(this.frenzyConfig));
        } catch (Exception ex) {
            Log.e(TAG, "Cannot initialize FreightFrenzy BotConfig", ex);
        }

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
            lift.setTargetPositionTolerance(positionToleranceLift);
        } catch (Exception ex) {
            Log.e(TAG, "Cannot initialize lift", ex);
        }

        try {
            turret = hwMap.get(DcMotorEx.class, "turret");
            turret.setDirection(DcMotor.Direction.REVERSE);
            turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            turret.setVelocity(0);
            turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            turret.setTargetPositionTolerance(positionToleranceTurret);
        } catch (Exception ex) {
            Log.e(TAG, "Cannot initialize turret", ex);
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

        try {
            intakeDropperServo =  hwMap.get(Servo.class, "intakeDropper");
            intakeDropperUp();
        } catch (Exception ex) {
            Log.e(TAG, "Cannot initialize Intake Dropper", ex);
        }

//        try {
//            tower =  hwMap.get(Servo.class, "tower");
//            initTower();
//        } catch (Exception ex) {
//            Log.e(TAG, "Cannot initialize tower servo", ex);
//        }

    }
//    public double getIntakeVoltage(){
//        VoltageSensor vs = hwMap.voltageSensor.get("intake");
//        if (vs == null){
//            return 0;
//        }
//        double voltage = vs.getVoltage();
//        return voltage;
//
//    }
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

    public void activateTurret(double velocity) {
        if (turret != null){
            //check for max range
            double currentPos = getTurretPosition();
            if (currentPos >= TURRET_POS_MAX_LEFT || currentPos <= TURRET_POS_MAX_RIGHT){
                return;
            }
            double turretVelocity = MAX_VELOCITY_REV*velocity/10;
            turret.setVelocity(turretVelocity);
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

    public int getTurretPosition(){
        return this.turret.getCurrentPosition();
    }

    @BotAction(displayName = "Lift level 3", defaultReturn = "")
    public void liftToLevel3(boolean block){
        liftLocation = LIFT_LEVEL_THREE;
        this.lift.setTargetPosition(LIFT_LEVEL_THREE);
        this.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.lift.setVelocity(MAX_VELOCITY_REV*LIFT_SPEED);
        if (block){
            while (owner.opModeIsActive() && this.lift.isBusy()){

            }
        }
    }

    public void liftToLevelMin(boolean block, boolean dirDown){
        if (dirDown == false){
            intakeDropperReady();
        }
        dropperTransportPosition();
        liftLocation = LIFT_MIN_EXTENSION;
        this.lift.setTargetPosition(LIFT_MIN_EXTENSION);
        this.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.lift.setVelocity(MAX_VELOCITY_REV*LIFT_SPEED);
        if (block){
            while (owner.opModeIsActive() && this.lift.isBusy()){
                if (dirDown && getLiftPosition() < LIFT_MIN_EXTENSION*2){
                    break;
                }
            }
        }
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

    @BotAction(displayName = "Lift Shared Hub", defaultReturn = "")
    public void liftSharedHub(boolean block){
        liftLocation = LIFT_SHARED_HUB;
        this.lift.setTargetPosition(LIFT_SHARED_HUB);
        this.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.lift.setVelocity(MAX_VELOCITY_REV*LIFT_SPEED);
        if (block){
            while (owner.opModeIsActive() && this.lift.isBusy()){

            }
        }
    }
    @BotAction(displayName = "Lift to lower", defaultReturn = "")
    public void liftToLower() {
        if (liftLocation != LIFT_UNDER_EXTENTION){
            liftLocation = LIFT_UNDER_EXTENTION;
            this.lift.setTargetPosition(LIFT_UNDER_EXTENTION);
            this.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            if (getLiftPosition() == LIFT_LEVEL_ONE) {
                this.lift.setVelocity(MAX_VELOCITY_REV * LIFT_SPEED_LOW);
            } else {
                this.lift.setVelocity(MAX_VELOCITY_REV * LIFT_SPEED);
            }

            while (owner.opModeIsActive() && this.lift.isBusy()){

            }
            resetDropper();
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
            // TODO: dropperServo.setPosition(frenzyConfig.getDropperPositionDrop());
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
            // TODO: dropperServo.setPosition(frenzyConfig.getDropperPositionReset());
        }
    }

    @BotAction(displayName = "Intake Dropper Up", defaultReturn = "")
    public void intakeDropperUp(){
        if (intakeDropperServo != null) {
            intakeDropperServo.setPosition(1);
        }
    }

    @BotAction(displayName = "Intake Dropper Down", defaultReturn = "")
    public void intakeDropperDown(){
        if (intakeDropperServo != null) {
            intakeDropperServo.setPosition(0);
        }
    }

    public void intakeDropperHalfWay(){
        if (intakeDropperServo != null) {
            intakeDropperServo.setPosition(0.6);
        }
    }

    public void intakeDropperReady(){
        if (intakeDropperServo != null) {
            intakeDropperServo.setPosition(0.7);
        }
    }


    @BotAction(displayName = "Main Tower to RED team hub", defaultReturn = "")
    public void turretToTeamHubRed(){
        this.turret.setTargetPosition(TURRET_POS_TEAMHUB_RED);
        this.turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.turret.setVelocity(MAX_VELOCITY_REV*TURRET_SPEED);
    }

    @BotAction(displayName = "Main Tower to BLUE team hub", defaultReturn = "")
    public void towerToTeamHubBlue(){
        this.turret.setTargetPosition(TURRET_POS_TEAMHUB_BLUE);
        this.turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.turret.setVelocity(MAX_VELOCITY_REV*TURRET_SPEED);
    }

    @BotAction(displayName = "Main Tower to RED shared hub", defaultReturn = "")
    public void towerToSharedHubRed(boolean block){
        this.turret.setTargetPosition(TURRET_POS_SHAREDHUB_RED);
        this.turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.turret.setVelocity(MAX_VELOCITY_REV*TURRET_SPEED);
        if (block){
            while (owner.opModeIsActive() && this.turret.isBusy()){

            }
        }
    }

    @BotAction(displayName = "Main Tower to BLUE shared hub", defaultReturn = "")
    public void towerToSharedHubBlue(boolean block){
        this.turret.setTargetPosition(TURRET_POS_SHAREDHUB_BLUE);
        this.turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.turret.setVelocity(MAX_VELOCITY_REV*TURRET_SPEED);
        if (block){
            while (owner.opModeIsActive() && this.lift.isBusy()){

            }
        }
    }

    @BotAction(displayName = "Reset tower", defaultReturn = "")
    public void resetTurret(){
        if (turret != null) {
            this.turret.setTargetPosition(TURRET_POS_CENTER);
            this.turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            this.turret.setVelocity(MAX_VELOCITY_REV*TURRET_SPEED_LOW);
        }
    }

    @BotAction(displayName = "Tower to hub from red warehouse", defaultReturn = "")
    public void towerToTeamHubFromAuto(){
        if (tower != null) {
            tower.setPosition(0.39);
        }
    }

    @BotAction(displayName = "Tower to hub from blue warehouse", defaultReturn = "")
    public void towerToTeamHubFromAutoWarehouseBlue(){
        if (tower != null) {
            tower.setPosition(0.62);
        }
    }

    @BotAction(displayName = "Tower to hub from red ducks", defaultReturn = "")
    public void towerToTeamHubFromAutoRedDucks(){
        if (tower != null) {
            tower.setPosition(0.35);
        }
    }

    @BotAction(displayName = "Tower to hub from blue ducks", defaultReturn = "")
    public void towerToTeamHubFromAutoBlueDucks(){
        if (tower != null) {
            tower.setPosition(0.35);
        }
    }

    public void dropperTransportPosition(){
        if (dropperServo != null) {
            dropperServo.setPosition(DROPPER_SERVO_POS_TRANSPORT);
            // TODO: dropperServo.setPosition(frenzyConfig.getDropperPositionTransport());
        }
    }

    @BotAction(displayName = "Start intake", defaultReturn = "")
    public void startIntake() {
        activateIntake(INTAKE_SPEED);
        // TODO: activateIntake(frenzyConfig.getIntakeSpeedIn());
        intakeRunning = true;
        intakeDropperDown();
//        dropperPickupPosition();
    }

    @BotAction(displayName = "Reverse intake", defaultReturn = "")
    public void reverseIntake() {
        activateIntake(INTAKE_SPEED_REVERSE);
        // TODO: activateIntake(frenzyConfig.getIntakeSpeedOut());
    }

    public void stopOuttake() {
        activateIntake(0);
    }

    public void smartStopIntake() {
        activateIntake(0);
        intakeDropperUp();
        delayWait(500);
    }

    public void smartStopIntakeAsync() {
        FrenzyIntake asyncIntake = new FrenzyIntake(this);
        Thread intakeThread = new Thread(asyncIntake);
        intakeThread.start();
    }

    @BotAction(displayName = "Stop intake", defaultReturn = "")
    public void stopIntake() {
        activateIntake(0);
    }

    @BotAction(displayName = "Start turntable blue", defaultReturn = "")
    public void startTurntableBlue() {
        activateRotatorLeft(0.35);
    }
    @BotAction(displayName = "Start turntable red", defaultReturn = "")
    public void startTurntableRed() {
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
        activateRotatorLeft(0.0);
    }
    public void toggleLight(boolean on){
        if (colorSensor != null && colorSensor instanceof SwitchableLight) {
            ((SwitchableLight) colorSensor).enableLight(on);
        }
    }

    @BotAction(displayName = "Drop to Team Hub Red", defaultReturn = "")
    public void dropToTeamHubRed() {
        extendToTeamHubRed();
        scoreAndFold();
    }

    public void extendToTeamHubRed() {
        liftToLevelMin(true, false);
        turretToTeamHubRed();
        liftToLevel3(true);
    }


    @BotAction(displayName = "Drop to Team Hub Blue", defaultReturn = "")
    public void dropToTeamHubBlue() {
        extendToTeamHubBlue();
        scoreAndFold();
    }

    public void extendToTeamHubBlue() {
        liftToLevelMin(true, false);
        towerToTeamHubBlue();
        liftToLevel3(true);
    }

    public void scoreAndFold() {
        dropElement();
        delayWait(800);
        prepDropperToMove();
        resetLift();
        resetTurret();
    }

    public void extendToTeamHubBlueAsync() {
        deliverToTeamHubAsync(AutoRoute.NAME_BLUE, false);
    }

    public void scoreToTeamHubBlueAsync() {
        deliverToTeamHubAsync(AutoRoute.NAME_BLUE, true);
    }

    public void extendToTeamHubRedAsync() {
        deliverToTeamHubAsync(AutoRoute.NAME_RED, false);
    }

    public void scoreToTeamHubRedAsync() {
        deliverToTeamHubAsync(AutoRoute.NAME_RED, true);
    }

    protected void deliverToTeamHubAsync(String opModeSide, boolean score){
        FrenzyLift asyncLift = new FrenzyLift(this, opModeSide, FrenzyLiftMode.TeamHub, score);
        Thread liftTread = new Thread(asyncLift);
        liftTread.start();
    }

    @BotAction(displayName = "Drop to Shared Hub Red", defaultReturn = "")
    public void dropToSharedHubRed() {
        liftToLevelMin(true, false);
        towerToSharedHubRed(false);
        liftSharedHub(true);
        dropElement();
        delayWait(800);
        prepDropperToMove();
        resetLift();
    }

    @BotAction(displayName = "Drop to Shared Hub Blue", defaultReturn = "")
    public void dropToSharedHubBlue(){
        liftToLevelMin(true, false);
        towerToSharedHubBlue(false);
        liftSharedHub(true);
        dropElement();
        delayWait(800);
        prepDropperToMove();
        resetLift();
    }

    public void dropToSharedHubRedAsync() {
        deliverToSharedHubAsync(AutoRoute.NAME_RED);
    }

    @BotAction(displayName = "Drop to Shared Hub Blue", defaultReturn = "")
    public void dropToSharedHubBlueAsync() {
        deliverToSharedHubAsync(AutoRoute.NAME_BLUE);
    }


    protected void deliverToSharedHubAsync(String opModeSide){
        FrenzyLift asyncLift = new FrenzyLift(this, opModeSide, FrenzyLiftMode.SharedHub, false);
        Thread liftTread = new Thread(asyncLift);
        liftTread.start();
    }

    public void resetLift() {
        liftToLevelMin(true, true);
        resetTurret();
        liftToLower();
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
        // TODO: moveAuto(frenzyConfig.getIntakeSpeedSlowIn());
        while (owner.opModeIsActive()){
            if (!isIntakeBoxEmpty()){
                Log.d(TAG, "Element is in the intake");
                gotIt = true;
                break;
            }
            if (runtime.milliseconds() > 1500){
                Log.d(TAG, "Ran out of time");
                break;
            }
        }

//        delayWait(200);
        activateIntake(0);
        stop();
        intakeRunning = false;
        intakeDropperUp();
        delayWait(500);

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
        double speedIncrement = -0.05;
        int maxLoops = 9;
        int loopDelayMs = 140;
        double maxSpeed = 0.38;
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

            activateRotatorLeft(currSpeed);

            this.delayWait(loopDelayMs);
        }

        activateRotatorLeft(0.0);
    }

    public boolean isIntakeRunning() {
        return intakeRunning;
    }

    public boolean isLiftEmergencyMode() {
        return liftEmergencyMode;
    }

    public void setLiftEmergencyMode(boolean liftEmergencyMode) {
        this.liftEmergencyMode = liftEmergencyMode;
    }
}
