package org.firstinspires.ftc.teamcode.bots;

import android.graphics.Color;
import android.nfc.Tag;
import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.internal.collections.SimpleGson;
import org.firstinspires.ftc.teamcode.CVRec.CVDetectMode;
import org.firstinspires.ftc.teamcode.CVRec.CVDetector;
import org.firstinspires.ftc.teamcode.CVRec.GameElement;
import org.firstinspires.ftc.teamcode.autonomous.AutoBase;
import org.firstinspires.ftc.teamcode.autonomous.AutoDot;
import org.firstinspires.ftc.teamcode.autonomous.AutoRoute;
import org.firstinspires.ftc.teamcode.calibration.FreightFrenzyConfig;
import org.firstinspires.ftc.teamcode.skills.DuckLoop;
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
//    private DistanceSensor sensorRange;

    private CRServo tapeMeasure = null;
    private Servo tapeUp = null;

    FreightFrenzyConfig frenzyConfig = null;

    private static final String TAG = "FrenzyBot";
    public static int LIFT_LEVEL_THREE = 1820;
    public static int LIFT_LEVEL_TWO = 1500;
    public static int LIFT_LEVEL_ONE = 1282;
    public static int LIFT_SHARED_HUB = 400;
    public static int LIFT_MIN_EXTENSION = 450;
    public static int LIFT_UNDER_EXTENTION = 0;
    public static int LIFT_ENDGAME = 180;

    protected static int TURRET_POS_CENTER = 0;
    protected static int TURRET_POS_MAX_LEFT = 840;  //red side team hub
    protected static int TURRET_POS_MAX_RIGHT = -728; //blue side team hub
    protected static int TURRET_POS_TEAMHUB_RED = 610;  //red side team hub
    protected static int TURRET_POS_TEAMHUB_BLUE = -661;  //red side team hub
    protected static int TURRET_POS_SHAREDHUB_RED = -634;  //red side team hub
    protected static int TURRET_POS_SHAREDHUB_BLUE = 578;  //red side team hub
    private static double TURRET_SPEED = 0.95;
    private static double TURRET_SPEED_LOW = 0.5;

    private boolean liftEmergencyMode = false; //if the lift is broken, operate with the intake
    private boolean isTeleOp = false;


    private int liftLocation = LIFT_UNDER_EXTENTION;
    private static double LIFT_SPEED = 0.95;
    private static double LIFT_SPEED_LOW = 0.7;
    protected static int positionToleranceLift = 15;
    protected static int positionToleranceTurret = 12;

    private int turretOffset = 0;
    private boolean turretOffsetDefined = false;

    NormalizedColorSensor colorSensor;

    // Dropper Servo positions
    private static double DROPPER_SERVO_POS_TRANSPORT = 0.5; // this is only to pick-up elements
    private static double DROPPER_SERVO_POS_TRANSPORT_DOWN = 0.65;
    private static double DROPPER_SERVO_POS_START = 0.3;  //default pos to start and transport
    private static double DROPPER_SERVO_POS_DROP = 1;

    // Detection
    CVDetector detector;
    String opModeSide = AutoRoute.NAME_RED;
    private GameElement detectedElement;


    //Intake
    private static double INTAKE_ELEMENT_MOVE_SPEED = 0.2;
    private static double INTAKE_SPEED = -0.2;
    private static double INTAKE_SPEED_REVERSE = 0.2;
    private boolean isGrabInProgress = false;

    private boolean intakeRunning = false;
    ElapsedTime spikeOccurredAt = new ElapsedTime();


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
//        try {
//            sensorRange = hwMap.get(DistanceSensor.class, "sensorRange");
//        } catch(Exception ex) {
//            Log.e(TAG, "Cannot initialize distanceSensor", ex);
//        }

        try {
            intakeDropperServo =  hwMap.get(Servo.class, "intakeDropper");
            if (isTeleOp() == false) {
                intakeDropperUp();
            }
        } catch (Exception ex) {
            Log.e(TAG, "Cannot initialize Intake Dropper", ex);
        }

        try {
            tapeMeasure =  hwMap.get(CRServo.class, "tapeMeasure");
            tapeMeasure.setDirection(DcMotorSimple.Direction.FORWARD);
        } catch (Exception ex) {
            Log.e(TAG, "Cannot initialize tape measure servo", ex);
        }

        try {
            tapeUp =  hwMap.get(Servo.class, "tapeUp");

            if (isTeleOp() == true) {
                initTapeMeasure();
            }
            else{
                tapeUp.setPosition(0); // for auto it must be up
            }

        } catch (Exception ex) {
            Log.e(TAG, "Cannot initialize tape measure servo", ex);
        }
    }

    public double getIntakeCurrent(){
        double curr = intake.getCurrent(CurrentUnit.AMPS);
        return curr;

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

    @BotAction(displayName = "Get Detection Result", defaultReturn = "C", isTerminator = false)
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

    public void activateTapeMeasure(double power) {
        if (tapeMeasure != null) {
            if (Math.abs(power) <= 0.2){
                tapeMeasure.setPower(0);
            }
            else {
                if (power < 0) {
                    tapeMeasure.setPower(-1);
                }
                else
                {
                    tapeMeasure.setPower(1);
                }
            }
        }
    }

    @BotAction(displayName = "InitTapeMeasure", defaultReturn = "", isTerminator = false)
    public void initTapeMeasure(){
        if (tapeUp != null){
            tapeUp.setPosition(0.43);
        }
    }

    public void moveTapeMeasureUpDown(double power) {
        if (tapeUp != null) {
            if (Math.abs(power) <= 0.2){
                return;
            }
            else {
                double MIN = 0;
                double MAX = 0.5;
                double increment = 0.025;
                double pos = tapeUp.getPosition();
                if (power < 0) {
                    double next = pos + increment;
                    if (next >= MAX){
                        next = MAX;
                    }
                    tapeUp.setPosition(next);
                }
                else
                {
                    double next = pos - increment;
                    if (next <= MIN){
                        next = MIN;
                    }
                    tapeUp.setPosition(next);
                }
            }
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

    @BotAction(displayName = "Lift level 3 Auto", defaultReturn = "", isTerminator = false)
    public void liftToLevel3Auto(){
        liftToLevel3(true);
    }

    public void liftToLevel3(boolean block){
        liftLocation = LIFT_LEVEL_THREE;
        this.lift.setTargetPosition(LIFT_LEVEL_THREE);
        this.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.lift.setVelocity(MAX_VELOCITY_REV*LIFT_SPEED);
        if (block){
            while (owner.opModeIsActive() && this.lift.isBusy()){

            }
            stopLift();
        }
    }

    public void liftToLevelMin(boolean block, boolean dirDown){
        if (dirDown == false){
            intakeDropperNeutral();
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
            stopLift();
        }
    }


    @BotAction(displayName = "Lift level 2 Auto", defaultReturn = "", isTerminator = false)
    public void liftToLevel2Auto(){
        liftToLevel2(true);
    }

    public void liftToLevel2(boolean block){
        liftLocation = LIFT_LEVEL_TWO;
        this.lift.setTargetPosition(LIFT_LEVEL_TWO);
        this.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.lift.setVelocity(MAX_VELOCITY_REV*LIFT_SPEED);
        if (block){
            while (owner.opModeIsActive() && this.lift.isBusy()){

            }
            stopLift();
        }
    }

    @BotAction(displayName = "Lift level 1 Auto", defaultReturn = "", isTerminator = false)
    public void liftToLevel1Auto(){
        liftToLevel1(true);
    }

    public void liftToLevel1(boolean block){
        liftLocation = LIFT_LEVEL_ONE;
        this.lift.setTargetPosition(LIFT_LEVEL_ONE);
        this.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.lift.setVelocity(MAX_VELOCITY_REV*LIFT_SPEED);
        if (block){
            while (owner.opModeIsActive() && this.lift.isBusy()){

            }
            stopLift();
        }
    }

    public void liftSharedHub(boolean block){
        liftLocation = LIFT_SHARED_HUB;
        this.lift.setTargetPosition(LIFT_SHARED_HUB);
        this.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.lift.setVelocity(MAX_VELOCITY_REV*LIFT_SPEED);
        if (block){
            while (owner.opModeIsActive() && this.lift.isBusy()){

            }
            stopLift();
        }
    }

    @BotAction(displayName = "Lift to lower", defaultReturn = "", isTerminator = false)
    public void liftToLower() {
        if (liftLocation != LIFT_UNDER_EXTENTION){
            this.lift.setTargetPosition(LIFT_UNDER_EXTENTION);
            this.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            if (getLiftPosition() == LIFT_LEVEL_ONE) {
                this.lift.setVelocity(MAX_VELOCITY_REV * LIFT_SPEED_LOW);
            } else {
                this.lift.setVelocity(MAX_VELOCITY_REV * LIFT_SPEED);
            }
            while (owner.opModeIsActive() && this.lift.isBusy()){

            }
            stopLift();

            liftLocation = LIFT_UNDER_EXTENTION;
        }
    }

    public boolean isLiftBusy(){
        return this.lift.isBusy();
    }

    private void stopLift(){
        this.lift.setPower(0);
        this.lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public int getLiftLocation(){
        return liftLocation;
    }

    @BotAction(displayName = "Drop element", defaultReturn = "", isTerminator = false)
    public void dropElement(){
        if (dropperServo != null) {
            dropperServo.setPosition(DROPPER_SERVO_POS_DROP);
            // TODO: dropperServo.setPosition(frenzyConfig.getDropperPositionDrop());
        }
    }

    @BotAction(displayName = "Reset dropper", defaultReturn = "", isTerminator = false)
    public void resetDropper(){
        prepDropperToMove();
    }

    @BotAction(displayName = "Prep dropper", defaultReturn = "", isTerminator = false)
    public void prepDropperToMove(){
        if (dropperServo != null) {
            dropperServo.setPosition(DROPPER_SERVO_POS_START);
            // TODO: dropperServo.setPosition(frenzyConfig.getDropperPositionReset());
        }
    }

    @BotAction(displayName = "Intake Dropper Up", defaultReturn = "", isTerminator = false)
    public void intakeDropperUp(){
        if (intakeDropperServo != null) {
            intakeDropperServo.setPosition(1);
        }
    }

    @BotAction(displayName = "Intake Dropper Down", defaultReturn = "", isTerminator = false)
    public void intakeDropperDown(){
        resetDropper();
        resetTurret();
        if (intakeDropperServo != null) {
            intakeDropperServo.setPosition(0);
        }
    }
    private void dropIntakeSimple() {
        if (intakeDropperServo != null) {
            intakeDropperServo.setPosition(0);
        }
    }

    public void intakeDropperHalfWay(){
        if (intakeDropperServo != null) {
            intakeDropperServo.setPosition(0.6);
        }
    }

    @BotAction(displayName = "Intake Dropper Neutral", defaultReturn = "", isTerminator = false)
    public void intakeDropperNeutral(){
        if (intakeDropperServo != null) {
            intakeDropperServo.setPosition(0.7);
        }
    }


    @BotAction(displayName = "Main Tower to RED team hub", defaultReturn = "", isTerminator = false)
    public void turretToTeamHubRed(){
        this.turret.setTargetPosition(TURRET_POS_TEAMHUB_RED + getTurretOffset());
        this.turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.turret.setVelocity(MAX_VELOCITY_REV*TURRET_SPEED);
    }

    @BotAction(displayName = "Main Tower to BLUE team hub", defaultReturn = "", isTerminator = false)
    public void towerToTeamHubBlue(){
        this.turret.setTargetPosition(TURRET_POS_TEAMHUB_BLUE + getTurretOffset());
        this.turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.turret.setVelocity(MAX_VELOCITY_REV*TURRET_SPEED);
    }

    public void towerToSharedHubRed(boolean block){
        this.turret.setTargetPosition(TURRET_POS_SHAREDHUB_RED + getTurretOffset());
        this.turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.turret.setVelocity(MAX_VELOCITY_REV*TURRET_SPEED);
        if (block){
            while (owner.opModeIsActive() && this.turret.isBusy()){

            }
        }
    }

    public void towerToSharedHubBlue(boolean block){
        this.turret.setTargetPosition(TURRET_POS_SHAREDHUB_BLUE + getTurretOffset());
        this.turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.turret.setVelocity(MAX_VELOCITY_REV*TURRET_SPEED);
        if (block){
            while (owner.opModeIsActive() && this.lift.isBusy()){

            }
        }
    }

    @BotAction(displayName = "Reset tower", defaultReturn = "", isTerminator = false)
    public void resetTurret(){
        if (turret != null) {
            int pos = this.turret.getCurrentPosition();
            if (Math.abs(pos) <= positionToleranceTurret){
                //do nothing;
                return;
            }
            this.turret.setTargetPosition(TURRET_POS_CENTER + getTurretOffset());
            this.turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            this.turret.setVelocity(MAX_VELOCITY_REV*TURRET_SPEED_LOW);
            while (owner.opModeIsActive() && this.turret.isBusy()){

            }
            this.turret.setPower(0);
            this.turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    @BotAction(displayName = "Tower to hub from red warehouse", defaultReturn = "", isTerminator = false)
    public void towerToTeamHubFromAuto(){
        this.turret.setTargetPosition(-360 + getTurretOffset());
        this.turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.turret.setVelocity(MAX_VELOCITY_REV*TURRET_SPEED);
    }

    @BotAction(displayName = "Tower to hub from blue warehouse", defaultReturn = "", isTerminator = false)
    public void towerToTeamHubFromAutoWarehouseBlue(){
        this.turret.setTargetPosition(277 + getTurretOffset());
        this.turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.turret.setVelocity(MAX_VELOCITY_REV*TURRET_SPEED);
    }

    @BotAction(displayName = "Tower to hub from red ducks", defaultReturn = "", isTerminator = false)
    public void towerToTeamHubFromAutoRedDucks(){
        this.turret.setTargetPosition(-293 + getTurretOffset());
        this.turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.turret.setVelocity(MAX_VELOCITY_REV*TURRET_SPEED);
    }

    @BotAction(displayName = "Tower to hub from blue ducks", defaultReturn = "", isTerminator = false)
    public void towerToTeamHubFromAutoBlueDucks(){
        this.turret.setTargetPosition(-325 + getTurretOffset());
        this.turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.turret.setVelocity(MAX_VELOCITY_REV*TURRET_SPEED);
    }

    public void dropperTransportPosition(){
        if (dropperServo != null) {
            dropperServo.setPosition(DROPPER_SERVO_POS_TRANSPORT);
            // TODO: dropperServo.setPosition(frenzyConfig.getDropperPositionTransport());
        }
    }

    public void dropperTransportPositionDown(){
        if (dropperServo != null) {
            dropperServo.setPosition(DROPPER_SERVO_POS_TRANSPORT_DOWN);
            // TODO: dropperServo.setPosition(frenzyConfig.getDropperPositionTransport());
        }
    }

    @BotAction(displayName = "Start intake", defaultReturn = "", isTerminator = false)
    public void startIntake() {
        resetDropper();
        activateIntake(INTAKE_SPEED);
        // TODO: activateIntake(frenzyConfig.getIntakeSpeedIn());
        intakeRunning = true;
        intakeDropperDown();
    }

    @BotAction(displayName = "Reverse intake", defaultReturn = "", isTerminator = false)
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

    @BotAction(displayName = "Stop intake", defaultReturn = "", isTerminator = false)
    public void stopIntake() {
        activateIntake(0);
    }

    @BotAction(displayName = "Start turntable blue", defaultReturn = "", isTerminator = false)
    public void startTurntableBlue() {
        activateRotatorLeft(0.35);
    }
    @BotAction(displayName = "Start turntable red", defaultReturn = "", isTerminator = false)
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

    @BotAction(displayName = "Start turntable blue gradual", defaultReturn = "", isTerminator = false)
    public void startTurntableBlueGradual() {
        duckLoopAsync(false, false);
    }
    @BotAction(displayName = "Start turntable red gradual", defaultReturn = "", isTerminator = false)
    public void startTurntableRedGradual() {
        duckLoopAsync(true, false);
    }

    @BotAction(displayName = "Start turntable blue auto gradual", defaultReturn = "", isTerminator = false)
    public void startTurntableBlueAutoGradual() {
        duckLoop(false, true);
    }
    @BotAction(displayName = "Start turntable red auto gradual", defaultReturn = "", isTerminator = false)
    public void startTurntableRedAutoGradual() {
        duckLoop(true, true);
    }

    @BotAction(displayName = "Stop turntable", defaultReturn = "", isTerminator = false)
    public void stopTurntable() {
        activateRotatorLeft(0.0);
    }
    public void toggleLight(boolean on){
        if (colorSensor != null && colorSensor instanceof SwitchableLight) {
            ((SwitchableLight) colorSensor).enableLight(on);
        }
    }

    @BotAction(displayName = "Extend to Team Hub Red Async", defaultReturn = "", isTerminator = false)
    public void extendToTeamHubRedAsync(){
        FrenzyLift frenzyLift = new FrenzyLift(this, AutoRoute.NAME_RED, FrenzyLiftMode.TeamHub, false);
        Thread thread = new Thread(frenzyLift);
        thread.start();
    }

    @BotAction(displayName = "Extend to Team Hub Blue Async", defaultReturn = "", isTerminator = false)
    public void extendToTeamHubBlueAsync(){
        FrenzyLift frenzyLift = new FrenzyLift(this, AutoRoute.NAME_BLUE, FrenzyLiftMode.TeamHub, false);
        Thread thread = new Thread(frenzyLift);
        thread.start();
    }


    @BotAction(displayName = "Extend to Team Hub Red", defaultReturn = "", isTerminator = false)
    public void extendToTeamHubRed() {
//        liftToLevelMin(false, false);
        intakeDropperNeutral();
        dropperTransportPosition();
        turretToTeamHubRed();
        liftToLevel3(true);
    }


    @BotAction(displayName = "Extend to Team Hub Blue", defaultReturn = "", isTerminator = false)
    public void extendToTeamHubBlue() {
//        liftToLevelMin(false, false);
        intakeDropperNeutral();
        dropperTransportPosition();
        towerToTeamHubBlue();
        liftToLevel3(true);
    }

    @BotAction(displayName = "Score and Fold", defaultReturn = "", isTerminator = false)
    public void scoreAndFold() {
        dropElement();
        delayWait(700);
        dropperTransportPositionDown();
        resetLift(false);
    }

    public void scoreAndFoldDyno() {
        dropElement();
        delayWait(700);
        dropperTransportPositionDown();
        resetLift(true);
    }


    @BotAction(displayName = "Score and Fold Async", defaultReturn = "", isTerminator = false)
    public void scoreAndFoldAsync() {
        FrenzyLift frenzyLift = new FrenzyLift(this,  true);
        Thread thread = new Thread(frenzyLift);
        thread.start();
    }


    public void extendToSharedHubRed() {
        intakeDropperNeutral();
        dropperTransportPosition();
        towerToSharedHubRed(false);
        liftSharedHub(true);
        intakeDropperUp();
    }

    public void extendToSharedHubBlue(){
        intakeDropperNeutral();
        dropperTransportPosition();
        towerToSharedHubBlue(false);
        liftSharedHub(true);
        intakeDropperUp();
    }

    public void extendToSharedHubRedAsync(){
        FrenzyLift frenzyLift = new FrenzyLift(this, AutoRoute.NAME_RED, FrenzyLiftMode.SharedHub, false);
        Thread thread = new Thread(frenzyLift);
        thread.start();
    }

    public void extendToSharedHubBlueAsync(){
        FrenzyLift frenzyLift = new FrenzyLift(this, AutoRoute.NAME_BLUE, FrenzyLiftMode.SharedHub, false);
        Thread thread = new Thread(frenzyLift);
        thread.start();
    }

    public void resetLift(boolean waitForLift) {
//        liftToLevelMin(true, true);
        liftToLower();
        if (waitForLift) {
            delayWait(100);
        }
        resetTurret();
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

//    public double getDistance() {
//        return sensorRange.getDistance(DistanceUnit.INCH);
//    }

    public boolean isIntakeBoxEmpty(){
        // Color based sensing
        float HValue = detectColor(telemetry, 0); // color method
        return HValue < 5;

//        Current based sensing
//        double val = getIntakeCurrent(); // Current Method
//        boolean spike = val>0.9;
//
//        if (isGrabInProgress) {
//            if(!spike){
//                if(spikeOccurredAt.milliseconds()>125){
//                    Log.d(TAG, "Spike ended properly");
//                    isGrabInProgress = false;
//                    return false;
//                } else{
//                    Log.d(TAG, "Spike ended early");
//                    isGrabInProgress = false;
//                    spikeOccurredAt.reset();
//                }
//            }
//        } else{
//            if(spike){
//                Log.d(TAG, "Spike started");
//                isGrabInProgress = true;
//                spikeOccurredAt.reset();
//            }
//        }
//        return true;

//        // Distance Sensor based detection
//        double dVal = getDistance();
//        return dVal >= 2.5;


    }

    @BotAction(displayName = "Grab Element", defaultReturn = "", isTerminator = false)
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
            if (runtime.milliseconds() > 3000){
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
        activateIntake(-0.15); //to avoid stuck balls
        delayWait(500);
        activateIntake(0);

        return gotIt;
    }

    @BotAction(displayName = "Time Check", defaultReturn = "", isTerminator = true)
    public Boolean timeCheckAuto(){
        if (this.owner instanceof AutoBase){
            double elapsedSeconds = ((AutoBase)this.owner).getOpModeElapsedTime();
            Log.d(TAG, String.format("shouldStopAuto called. %.2f seconds left", elapsedSeconds));
            return elapsedSeconds > 23; //time before the end of the cycle should not exceed 7 secs
        }
        return Boolean.FALSE;
    }

    public void goDucks(boolean red){
        for(int i = 0; i < 12; i++){
            duckLoop(red, false);
            delayWait(600);
        }
    }

    public void duckLoopAsync(boolean red, boolean auto) {
        DuckLoop duckLoop = new DuckLoop(this, red, auto);
        Thread thread = new Thread(duckLoop);
        thread.start();
    }

    public void duckLoop(boolean red, boolean auto){
        double startSpeed = -0.2;
        double speedIncrement = -0.05;
        int maxLoops = 11;
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

    public void liftToLevelEndgame(){
        this.lift.setTargetPosition(LIFT_ENDGAME);
        this.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.lift.setVelocity(MAX_VELOCITY_REV*LIFT_SPEED);
        while (owner.opModeIsActive() && this.lift.isBusy()){

        }
        stopLift();

        dropperTransportPositionDown();
        intakeDropperUp();
    }

    public double getTapePosition() {
        double pos = tapeUp.getPosition();
        return pos;
    }

    public void liftTapeEndgame() {
        tapeUp.setPosition(0.285);
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

    public boolean isTeleOp() {
        return isTeleOp;
    }

    public void setTeleOp(boolean teleOp) {
        isTeleOp = teleOp;
    }

    public int getTurretOffset() {
        return turretOffset;
    }

    public void defineTurretOffset() {
        if (!turretOffsetDefined) {
            this.turretOffset = this.turret.getCurrentPosition();
            turretOffsetDefined = true;
        }
    }

    public boolean isTurretOffsetDefined() {
        return turretOffsetDefined;
    }

}
