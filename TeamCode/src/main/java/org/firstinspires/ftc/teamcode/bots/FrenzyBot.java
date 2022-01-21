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
    public static int LIFT_LEVEL_THREE = 2220;
    public static int LIFT_LEVEL_TWO = 1945;
    public static int LIFT_LEVEL_ONE = 1720;
    public static int LIFT_SHARED_HUB = 260;
    public static int LIFT_NO_EXTENSION = 0;
    public static int LIFT_MIN_EXTENSION = 300;
    public static int LIFT_UNDER_EXTENTION = 5;

    private boolean liftEmergencyMode = false; //if the lift is broken, operate with the intake

    //New lift vals: TOP - 1755. MIDDLE -1482. LOW - 1282

    private int liftLocation = LIFT_NO_EXTENSION;
    private static double LIFT_SPEED = 0.95;
    private static double LIFT_SPEED_LOW = 0.7;
    protected static int positionToleranceLift = 15;
    protected static int positionToleranceTurret = 15;

    NormalizedColorSensor colorSensor;

    // Dropper Servo positions
    private static double DROPPER_SERVO_POS_TRANSPORT = 0.2; // this is only to pick-up elements
    private static double DROPPER_SERVO_POS_START = 0.0;  //default pos to start and transport
    private static double DROPPER_SERVO_POS_DROP = 0.85;

    // Detection
    CVDetector detector;
    String opModeSide = AutoRoute.NAME_RED;
    private GameElement detectedElement;


    //Intake
    private static double INTAKE_ELEMENT_MOVE_SPEED = 0.2;
    private static double INTAKE_SPEED = -0.8;
    private static double INTAKE_SPEED_REVERSE = 0.55;

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

        try {
            tower =  hwMap.get(Servo.class, "tower");
            initTower();
        } catch (Exception ex) {
            Log.e(TAG, "Cannot initialize tower servo", ex);
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

    public void activateTurret(double velocity) {
        if (turret != null){
            turret.setVelocity(MAX_VELOCITY_REV*velocity);
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

    public void liftToLevelMin(boolean block){
        liftLocation = LIFT_LEVEL_THREE;
        this.lift.setTargetPosition(LIFT_MIN_EXTENSION);
        this.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.lift.setVelocity(MAX_VELOCITY_REV*LIFT_SPEED);
        if (block){
            while (owner.opModeIsActive() && this.lift.isBusy()){

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
    public void liftSharedHub(){
        liftLocation = LIFT_SHARED_HUB;
        this.lift.setTargetPosition(LIFT_SHARED_HUB);
        this.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.lift.setVelocity(MAX_VELOCITY_REV*LIFT_SPEED);
    }
    @BotAction(displayName = "Lift to lower", defaultReturn = "")
    public void liftToLower() {
        if (liftLocation != LIFT_UNDER_EXTENTION){
            //reset dropper before retracting the lift all the way
            if (dropperServo.getPosition() > 0.5) {
                resetDropper();
                delayWait(999);
            } else {
                resetDropper();
            }
            liftLocation = LIFT_UNDER_EXTENTION;
            this.lift.setTargetPosition(LIFT_UNDER_EXTENTION);
            this.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            if (getLiftPosition() == LIFT_LEVEL_ONE) {
                this.lift.setVelocity(MAX_VELOCITY_REV * LIFT_SPEED_LOW);
            } else {
                this.lift.setVelocity(MAX_VELOCITY_REV * LIFT_SPEED);
            }
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

    @BotAction(displayName = "Init tower", defaultReturn = "")
    public void initTower(){
        if (tower != null) {
            tower.setPosition(0.5);
        }
    }

    @BotAction(displayName = "Main Tower to RED team hub", defaultReturn = "")
    public void towerToTeamHubRed(){
        if (tower != null) {
            tower.setPosition(0.75);
        }
    }

    @BotAction(displayName = "Main Tower to BLUE team hub", defaultReturn = "")
    public void towerToTeamHubBlue(){
        if (tower != null) {
            tower.setPosition(0.25);
        }
    }

    @BotAction(displayName = "Main Tower to RED shared hub", defaultReturn = "")
    public void towerToSharedHubRed(){
        if (tower != null) {
            tower.setPosition(0.33);
            delayWait(1000);
            tower.setPosition(0.32);
        }
    }

    @BotAction(displayName = "Main Tower to BLUE shared hub", defaultReturn = "")
    public void towerToSharedHubBlue(){
        if (tower != null) {
            tower.setPosition(0.75);
        }
    }

    @BotAction(displayName = "Reset tower", defaultReturn = "")
    public void resetTower(){
        if (tower != null) {
            double currentPos = tower.getPosition();
            double diff = 0.5 - currentPos;
            double targetPos = 0.51;
            if (diff > 0){
                targetPos = 0.49;
            }
            tower.setPosition(targetPos);
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
        delayWait(800);
        activateIntake(-0.15);
        delayWait(800);
        activateIntake(0);
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
        liftToLevelMin(true);
        dropperTransportPosition();
        towerToTeamHubRed();
        delayWait(500);
        liftToLevel3(true);
        dropElement();
        delayWait(800);
        resetDropper();
        resetLift();
    }

    @BotAction(displayName = "Async Drop to TH RED", defaultReturn = "")
    public void dropToTeamHubRedAsync() {
        deliverToTeamHubAsync(AutoRoute.NAME_RED);
    }

    @BotAction(displayName = "Drop to Team Hub Blue", defaultReturn = "")
    public void dropToTeamHubBlue() {
        liftToLevelMin(true);
        dropperTransportPosition();
        towerToTeamHubBlue();
        delayWait(500);
        liftToLevel3(true);
        dropElement();
        delayWait(800);
        resetDropper();
        resetLift();
    }

    @BotAction(displayName = "Async Drop to TH BLUE", defaultReturn = "")
    public void dropToTeamHubBlueAsync() {
        deliverToTeamHubAsync(AutoRoute.NAME_BLUE);
    }

    protected void deliverToTeamHubAsync(String opModeSide){
        FrenzyLift asyncLift = new FrenzyLift(this, opModeSide, FrenzyLiftMode.TeamHub);
        Thread liftTread = new Thread(asyncLift);
        liftTread.start();
    }

    @BotAction(displayName = "Drop to Shared Hub Red", defaultReturn = "")
    public void dropToSharedHubRed() {
        dropToSharedHub();
    }

    @BotAction(displayName = "Drop to Shared Hub Blue", defaultReturn = "")
    public void dropToSharedHubBlue() {
        dropToSharedHub();
    }

    public void dropToSharedHubRedAsync() {
        deliverToSharedHubAsync(AutoRoute.NAME_RED);
    }

    @BotAction(displayName = "Drop to Shared Hub Blue", defaultReturn = "")
    public void dropToSharedHubBlueAsync() {
        deliverToSharedHubAsync(AutoRoute.NAME_BLUE);
    }

    public void dropToSharedHub() {
        //liftToLevelMin();
        //delayWait(400);
        //towerToSharedHubRed();
        //delayWait(1000);
        initTower();
        liftSharedHub();
        delayWait(700);
        dropElement();
        delayWait(800);
        resetDropper();
        resetLift();
    }

    protected void deliverToSharedHubAsync(String opModeSide){
        FrenzyLift asyncLift = new FrenzyLift(this, opModeSide, FrenzyLiftMode.SharedHub);
        Thread liftTread = new Thread(asyncLift);
        liftTread.start();
    }

    public void resetLift() {
        liftToLevelMin(true);
        resetTower();
        liftToLower();
        delayWait(500);
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
        delayWait(800);
        activateIntake(-0.15);
        delayWait(700);
        activateIntake(0);

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
