package org.firstinspires.ftc.teamcode.robots.taubot.subsystem;

import static org.firstinspires.ftc.teamcode.robots.taubot.util.Constants.MAX_CHASSIS_LENGTH;
import static org.firstinspires.ftc.teamcode.robots.taubot.util.Utils.servoDenormalize;
import static org.firstinspires.ftc.teamcode.robots.taubot.util.Utils.servoNormalize;
import static org.firstinspires.ftc.teamcode.robots.taubot.util.Constants.INCHES_PER_METER;
import static org.firstinspires.ftc.teamcode.robots.taubot.util.Utils.withinError;
import static org.firstinspires.ftc.teamcode.util.utilMethods.futureTime;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.teamcode.robots.taubot.ConeStack;
import org.firstinspires.ftc.teamcode.robots.taubot.Field;
import org.firstinspires.ftc.teamcode.robots.taubot.FieldThing;
import org.firstinspires.ftc.teamcode.robots.taubot.PowerPlay_6832;
import static org.firstinspires.ftc.teamcode.robots.taubot.PowerPlay_6832.active;
import org.firstinspires.ftc.teamcode.robots.taubot.simulation.DcMotorExSim;

import org.firstinspires.ftc.teamcode.robots.taubot.simulation.DistanceSensorSim;
import org.firstinspires.ftc.teamcode.robots.taubot.simulation.ServoSim;

import org.firstinspires.ftc.teamcode.robots.taubot.util.Utils;
import org.firstinspires.ftc.teamcode.robots.taubot.util.Constants;
import org.firstinspires.ftc.teamcode.statemachine.Stage;
import org.firstinspires.ftc.teamcode.statemachine.StateMachine;
import org.firstinspires.ftc.teamcode.util.PIDController;
import org.firstinspires.ftc.teamcode.util.Vector3;

import java.util.LinkedHashMap;
import java.util.Map;

@Config(value = "PPCrane")
public class Crane implements Subsystem {

    //control constants
    public static double PICK_UP_VELOCITY = 0.5;
    public static double HEIGHT_AFTER_PICKING_UP_CONE = 15;

    public static int SHOULDER_START_ANGLE = 0;
    public static int BULB_HOME_PWM = 1500;

    public static double SHOULDER_TICKS_PER_DEGREE = 15.7;  //from Proteus todo verify it works
    // This initial measurement is the range of motion from fully up (7.25 degrees from vertical) to horizontal, divided by that angle range
    // note the arm was moved by hand, not under motor power, and the angle encoder was not properly secured
    public static double SHOULDER_DEG_MIN = -10; // degrees down from horizontal - negative angles are counter clockwise while looking at the left side of the bot
    public static double SHOULDER_DEG_MAX = 79; //max elevation of shoulder when stalled up - measured by inclinometer
    public static double SHOULDER_DIRECT_TICKS_PER_DEGREE = (1937-88)/(SHOULDER_DEG_MAX); //todo verify/update when sensor secured and robot is more burned in - before tuning precision articulation

    public static double SHOULDER_TICK_MAX = 1849;

    public static double EXTEND_TICKS_PER_METER = 3700/1.6129; //todo verify this is still true

    public static void setShoulderImuEnable(boolean shoulderImuEnable) {
        SHOULDER_IMU_ENABLE = shoulderImuEnable;
    }

    public static boolean SHOULDER_IMU_ENABLE = true; //set false to disable IMU calls while testing off-robot subsystems

    public static double kF = 0.2;
    public static PIDCoefficients SHOULDER_PID = new PIDCoefficients(0.05, 0.005, 0.0);
    public static double SHOULDER_MAX_PID_OUTPUT = 1;

    public static double FOLD_SHOULDER_POSITION = 11;
    public static double SHOULDER_MIN_PID_OUTPUT = -1;
    public static double SHOULDER_TOLERANCE = 1;
    public static double SHOULDER_POWER = 1.0;
    public static double SHOULDER_ADJUST = 13;
    public static double EXTEND_ADJUST = .05;
    public static double TURRET_ADJUST = 20;
    public static double HEIGHT_ADJUST = 13;
    public static double DISTANCE_ADJUST = 30;

    public static double SHOULDER_ERROR_MAX = 6;
    public static double EXTENSION_ERROR_MAX = 0.05;
    public static double TURRET_ERROR_MAX = 4;

    public static double kE = 0.0;
    public static PIDCoefficients EXTENDER_PID = new PIDCoefficients(10, 0, 0.005);
    public static double EXTEND_MAX_PID_OUTPUT = 1;
    public static double EXTEND_MIN_PID_OUTPUT = -1;
    public static double EXTENDER_TOLERANCE = 1;
    public static double EXTENDER_POWER = 1.0;
    public static double EXTENDER_TICS_MIN = 0;
    public static double EXTENDER_TICS_MAX = 3700; // of the robot
    boolean EXTENDER_CALIBRATE_MAX = false; //keep false except if calibrating EXTENDER_TICS_MAX

    public static double BULB_OPEN_POS = 1300;
    public static double BULB_CLOSED_POS = 1500;

    public static double TRANSFER_SHOULDER_ANGLE = 50;  //angle at which transfer occurs
    public static double TRANSFER_SHOULDER_FLIPANGLE = 60; //angle that puts transfer plate at best angle to receive bulb gripper
    public static double TRANSFER_ARM_LENGTH = 0.05;

    public static double OLD_TRANSFER_SHOULDER_ANGLE = 50;  //angle at which transfer occurs
    public static double OLD_TRANSFER_SHOULDER_FLIPANGLE = 85; //causes the gripperflipper to flip when the angle is high and the turret turns enough or the robot accellerates
    public static double OLD_TRANSFER_ARM_LENGTH = 0.05;

    public static double SAFE_SHOULDER_ANGLE = 30;
    public static double SAFE_ARM_LENGTH = 0.05;

    public static double HOME_SAFE_SHOULDER_ANGLE = 15;

    public static double nudgeStickProportion= -12;
    public static double nudgeStickModifierOffset = 1800;

    public static double nudgeTuckValue = 1080; //maximum tuckage
    public static double nudgeMaxValue = 1600; //maximum extension
    public double nudgeRange = nudgeMaxValue - nudgeTuckValue;
    public double nudgeCenter = nudgeRange/2 + nudgeTuckValue;

    public static int FLIPPER_HOME = 900;
    public static int FLIPPER_FLIP = 1716;
    public static int FLIPPER_TENSION = 1900;
    public static int FLIPPER_REST = 1300;

    public static final double DISTANCE_SENSOR_TO_ELBOW = 0.33;
    public static final double GRIPPER_HEIGHT = 0.23;
    public static final double Y_LEEWAY = 0.05;
    public static final double X_LEEWAY = 0.02;
    public static final double ELBOW_HEIGHT = 0.24;
    public static final double CRANE_LENGTH = .3683;
    public static double extenderPwr = 1.0;
    double extendCorrection = 0;
    double extenderTargetPos = 0;
    private boolean extenderActivePID = true;
    private boolean shoulderActivePID = true;

    private boolean craneCalibrationEnabled = false;

    public Servo bulbServo;
    public DcMotorEx extenderMotor;public DcMotorEx shoulderMotor;
    public DcMotorEx turretMotor;
//    public DcMotor shoulderAngleEncoder;

    private PIDController shoulderPID;
    private PIDController extendPID;

    private boolean bulbGripped;

    private Robot robot;


    private Articulation articulation = Articulation.init;

    boolean USE_MOTOR_SMOOTHING = true;

    private Servo nudgeStickServo;

    private Servo flipperServo;

    private DistanceSensor nudgeDistanceSensor;
    private double nudgeDistance;

    BNO055IMU shoulderImu;
    BNO055IMU turretImu;

    Orientation angles;
    Acceleration gravity;

    Orientation turretAngles;
    Acceleration turretGravity;

    DigitalChannel turretIndex;

    StateMachine currentStateMachine = Utils.getStateMachine(new Stage()).addState(()->{return true;}).build();

    public Crane(HardwareMap hardwareMap, Robot robot, boolean simulated) {
        this.robot = robot;
        extenderTargetPos = 0;
        shoulderTargetAngle = 0;
        if (simulated) {
            setShoulderImuEnable(false);
            shoulderMotor = new DcMotorExSim(USE_MOTOR_SMOOTHING);
            extenderMotor = new DcMotorExSim(USE_MOTOR_SMOOTHING);
            turretMotor = new DcMotorExSim(USE_MOTOR_SMOOTHING);
//            shoulderAngleEncoder = new DcMotorExSim(USE_MOTOR_SMOOTHING);
            nudgeDistanceSensor = new DistanceSensorSim(0);
            bulbServo = new ServoSim();
            nudgeStickServo = new ServoSim();
        } else {
            shoulderMotor = hardwareMap.get(DcMotorEx.class, "shoulder");
//            shoulderAngleEncoder = hardwareMap.get(DcMotorEx.class, "shoulderAngleEncoder"); //just a REV shaft encoder - no actual motor
            extenderMotor = hardwareMap.get(DcMotorEx.class, "extender");
            turretMotor = hardwareMap.get(DcMotorEx.class, "turret");
            shoulderMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            extenderMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            extenderMotor.setTargetPosition(0);
            shoulderMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            extenderMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            shoulderMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            bulbServo = hardwareMap.get(Servo.class, "servoGripper");
            nudgeStickServo = hardwareMap.get(Servo.class, "nudgeSwivel");
            //nudgeDistanceSensor = hardwareMap.get(DistanceSensor.class, "nudgeDist");
            //turretIndex = hardwareMap.get(DigitalChannel.class, "turretIndex");
            //turretIndex.setMode(DigitalChannel.Mode.INPUT);

        }
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        shoulderImu = hardwareMap.get(BNO055IMU.class, "shoulderIMU");
        shoulderImu.initialize(parameters);
        turretImu = hardwareMap.get(BNO055IMU.class, "turretIMU");
        turretImu.initialize(parameters);

        turretImu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        extendPID = new PIDController(0,0,0);
        extendPID.setOutputRange(EXTEND_MIN_PID_OUTPUT, EXTEND_MAX_PID_OUTPUT);
        shoulderPID = new PIDController(0,0,0);
        shoulderPID.setOutputRange(SHOULDER_MIN_PID_OUTPUT,SHOULDER_MAX_PID_OUTPUT);
        shoulderPID.setIntegralCutIn(10);
        shoulderPID.enableIntegralZeroCrossingReset(false);
        shoulderActivePID = false;
        extenderActivePID = false;
        fieldPositionTarget = new Vector3(robot.driveTrain.getPoseEstimate().getX()+6,robot.driveTrain.getPoseEstimate().getY(),8);
        articulate(Articulation.init);

        flipperServo = hardwareMap.get(Servo.class, "flipperServo");

        goTargetInd = 0;
        homeInd = 0;
        coneCycleStage = 0;
        coneStackStage = 0;
        pickupConeStage = 0;
        dropConeStage = 0;
    }

    public void resetCrane(Constants.Position start){
        if(PowerPlay_6832.gameState.equals(PowerPlay_6832.GameState.AUTONOMOUS) || PowerPlay_6832.gameState.equals(PowerPlay_6832.GameState.TEST)){
            fieldPositionTarget = new Vector3(start.getPose().getX()+6,start.getPose().getY(),9);
        }else if(PowerPlay_6832.gameState.equals(PowerPlay_6832.GameState.TELE_OP)){
            fieldPositionTarget = new Vector3(home.x+robot.turret.getTurretPosition().getX(),home.y+robot.turret.getTurretPosition().getY(),home.z);
        }
    }

    int calibrateStage=0;
    double futureTime;
    double runShoulderAmp;
    double runExtendAmp;
    int extendMaxTics;

    public static double CALIBRATE_STALL_AMP = 3.75;

    public boolean calibrate(){
        // to calibrate we want the arm to be fully retracted and the shoulder
        // to be fully up at the physical stop as a repeatable starting position
        switch (calibrateStage) {
            case 0:
                enableShoulderPID();
                articulate(Articulation.manual);
                robot.driveTrain.articulate(DriveTrain.Articulation.lockWheels);
                robot.driveTrain.setChassisLength(MAX_CHASSIS_LENGTH);
                calibrated = false; //allows us to call calibration mid-match in an emergency
                //operator instruction: physically push arm to about 45 degrees and extend by 1 slide before calibrating
                //shoulder all the way up and retract arm until they safely stall
                extenderActivePID = false;
                robot.driveTrain.extend();
                extenderMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                extenderMotor.setPower(-0.4);
                setShoulderTargetAngle(SAFE_SHOULDER_ANGLE);
                futureTime = futureTime(.25);
                calibrateStage++;
                break;

            case 1:
                if(System.nanoTime() > futureTime){
                    //sample low load amps
                    runExtendAmp = extenderMotor.getCurrent(CurrentUnit.AMPS);
                    calibrateStage++;
                }
                break;

            case 2:
                // both motors are stalled
                if(extenderMotor.getCurrent(CurrentUnit.AMPS) > CALIBRATE_STALL_AMP){
                    extenderMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    extenderMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    extenderMotor.setPower(-0.1); //barely stop it from extending
                    //enable PID on shoulder and rotate down to horizontal
                    calibrateStage++;
                }
                break;
            case 3:
                calibrateStage++;
                break;

            case 4: //enable extender PID to zero position - give 2 seconds to retract a good bit
                extenderMotor.setPower(0.0);
                extenderTargetPos = 0;
                extenderActivePID = true;
                futureTime = futureTime(0.5);
                calibrateStage++;
                break;

            case 5:
                if(robot.turret.calibrate()){
                    calibrateStage++;
                }
                break;

            case 6:
                if (System.nanoTime()>futureTime) {
                    enableAllPID();
                    setShoulderTargetAngle(FOLD_SHOULDER_POSITION);
                    articulate(Articulation.init);
                    robot.driveTrain.articulate(DriveTrain.Articulation.lock);
                    calibrateStage = 0;
                    extenderActivePID = true;
                    shoulderActivePID = true;
                    calibrated = true;
                    return true;
                }
                break;

        }
        return false;
    }


    boolean calibrated = true;

    FieldThing targetPole;
    FieldThing source;

    private void setFlipperPosition(int ticks){
        flipperServo.setPosition(servoNormalize(ticks));
    }

    public void flipToHome(){
        flipperPos = 0;
    }

    public void flipToFlip(){
        flipperPos = 1;
    }

    public void flipToRest(){
        flipperPos = 2;
    }

    public void flipToTension(){
        flipperPos = 3;
    }
    int flipperPos = 0;
    private void updateFlipperPosition(){
        switch(flipperPos){
            case 0:
                setFlipperPosition(FLIPPER_HOME);
                break;
            case 1:
                setFlipperPosition(FLIPPER_FLIP);
                break;
            case 2:
                setFlipperPosition(FLIPPER_REST);
                break;
            case 3:
                setFlipperPosition(FLIPPER_TENSION);
                break;
        }
    }

    public void updateScoringPattern(){
        targetPole = robot.field.getPatternObject();
        source = robot.field.getConeSource();
    }

    double shoulderCorrection = 0;
    double shoulderPwr = 1;
    double shoulderTargetAngle = 0;

    //keeps track of current nudge position
    double nudgeTarget;

    boolean nudgeTuck = true;
    public void updateNudgeStick(){
        nudgeTarget = servoNormalize(Range.clip(nudgeStickProportion*getShoulderAngle() + nudgeStickModifierOffset, nudgeTuckValue, nudgeMaxValue));
        if(!nudgeTuck)
            nudgeStickServo.setPosition(nudgeTarget);
        else
            nudgeStickServo.setPosition(servoNormalize(nudgeTuckValue));
    }

    public void tuckNudgeStick(){
        nudgeTuck = true;
    }

    public void extendNudgeStick(){
        nudgeTuck = false;
    }

    public boolean isBulbGripped(){
        return bulbGripped;
    }
    public void movePIDShoulder(double Kp, double Ki, double Kd, double currentTicks, double targetTicks) {

        //initialization of the PID calculator's output range, target value and multipliers
        shoulderPID.setOutputRange(SHOULDER_MIN_PID_OUTPUT, SHOULDER_MAX_PID_OUTPUT);
        shoulderPID.setPID(Kp, Ki, Kd, (gravityModiferForCraneExtensionAndAngle) -> kF * getExtendMeters()/2 * Math.cos(Math.toRadians(gravityModiferForCraneExtensionAndAngle)));
        shoulderPID.setSetpoint(targetTicks);
        shoulderPID.enable();

        //initialization of the PID calculator's input range and current value
        shoulderPID.setInput(currentTicks);

        //calculates the correction to apply
        shoulderCorrection = shoulderPID.performPID();

        //moves elbow with the correction applied
        if(System.nanoTime() > stallTimer) {
            if (shoulderAmps < 4) {
                shoulderMotor.setPower(shoulderCorrection);
            } else {
                stallTimer = futureTime(5.0);
            }
        }else {
            shoulderMotor.setPower(0);
        }
    }

    long stallTimer = 0;

    public void movePIDExtend(double Kp, double Ki, double Kd, double currentTicks, double targetTicks) {

        //todo - probably don't need our own PID - can use built in PID
        //initialization of the PID calculator's output range, target value and multipliers
        extendPID.setOutputRange(EXTEND_MIN_PID_OUTPUT, EXTEND_MAX_PID_OUTPUT);
        extendPID.setPID(Kp, Ki, Kd);
        extendPID.setSetpoint(targetTicks);
        extendPID.enable();

        //initialization of the PID calculator's input range and current value
        //extendPID.setInputRange(0, 360);
        //extendPID.setContinuous();
        extendPID.setInput(currentTicks);

        //calculates the correction to apply
        extendCorrection = extendPID.performPID();

        //performs the extension with the correction applied
        extenderMotor.setPower(extendCorrection);
    }

    public enum Articulation {
        manual,
        manualDrive,
        dropCone,
        scoreCone,
        home,
        coneStackRight,
        coneStackLeft,
        transfer,
        transferStage2,
        lock,
        postTransfer,
        init,
        dropConeReturnToTransfer,
        autonDrive,
        lockToHome
    }

    public Articulation getArticulation() {
        return articulation;
    }


    int dropThenTransferStage = 0;
    long dropThenTransferTimer = 0;
    public boolean dropThenTransfer(){
        switch (dropThenTransferStage){
            case 0:
                release();
                dropThenTransferTimer = futureTime(0.5);
                flipToFlip();
                dropThenTransferStage++;
                break;
            case 1:
                if(System.nanoTime() > dropThenTransferTimer && goHome()) {
                    dropThenTransferTimer = futureTime(0.4);
                    dropThenTransferStage++;
                }
                break;
            case 2:
                if(System.nanoTime() > dropThenTransferStage){
                    setShoulderTargetAngle(TRANSFER_SHOULDER_FLIPANGLE);
                    setExtendTargetPos(TRANSFER_ARM_LENGTH);
                    dropThenTransferTimer = futureTime(0.7);
                    flipToHome();
                    dropThenTransferStage++;
                    fieldPositionTarget = new Vector3(-4,0,10).add(robotPosition);
                }
                break;
            case 3:
                dropThenTransferStage = 0;
                return true;
        }
        return false;
    }

    public Articulation articulate(Articulation target){
        articulation = target;

        switch(articulation){
            case lockToHome:
                if(goLock()){
                    articulation = Articulation.lock;
                }
                break;
            case dropConeReturnToTransfer:
                if(dropThenTransfer()){
                    articulation = Articulation.transfer;
                }
                break;
            case transferStage2:
                setShoulderTargetAngle(0);
                break;
            case lock: //if the robot is driving all cranes should go into a safe position
                if(deltaGripperPosition != null && robotPosition != null) {
                    fieldPositionTarget = deltaGripperPosition.add(robotPosition);
                }
                robot.turret.articulate(Turret.Articulation.transfer);
                setShoulderTargetAngle(SAFE_SHOULDER_ANGLE);
                setExtendTargetPos(SAFE_ARM_LENGTH);
                break;
            case autonDrive:
                fieldPositionTarget = deltaGripperPosition.add(robotPosition);
                setShoulderTargetAngle(SAFE_SHOULDER_ANGLE);
                setExtendTargetPos(SAFE_ARM_LENGTH);
                break;
            case transfer: //this just puts the transfer plate in the right position to receive a cone from underarm
                if(Transfer()) articulation = Articulation.manual;
                break;
            case init:
                robot.turret.articulate(Turret.Articulation.lockToZero);
                break;
            case postTransfer:
                if(postTransfer()){
                    articulation = Articulation.manual;
                }
                break;
            case manual:
                //do nothing
                break;
            case manualDrive:
                robot.turret.articulate(Turret.Articulation.runToAngle);
                holdTarget(fieldPositionTarget.x,fieldPositionTarget.y,fieldPositionTarget.z);
                break;
            case dropCone:
                pickupConeStage = 0;
                updateScoringPattern();
                if(dropCone(source)){
                    articulation = Articulation.manualDrive;
                }
                break;

            case home:
                pickupConeStage = 0;
                dropConeStage = 0;
                if(goHome()){
                    articulation = Articulation.manualDrive;
                }
                break;

            case scoreCone:
                dropConeStage = 0;
                updateScoringPattern();
                //if(goToFieldthing(targetPole)){
                if(pickupCone(targetPole)){
                    articulation = Articulation.manualDrive;
                }

                break;
            case coneStackRight:
                if(coneStack(true)){
                    articulation = Articulation.manualDrive;
                }
                break;
            case coneStackLeft:
                if(coneStack(false)){
                    articulation = Articulation.manualDrive;
                }
                break;
            default:
                return target;
        }
        return target;
    }

    public void resetArticulations(){
        articulation = Articulation.manual;
        transferStage = 0;
        dropThenTransferStage = 0;
        goTargetInd = 0;
        pickupConeStage = 0;
        dropConeStage = 0;
        homeInd = 0;
        calibrateStage = 0;
        coneCycleStage = 0;
        postTransferStage = 0;
    }
    int transferStage = 0;
    long transferTimer;

    boolean craneTransferReady = false;

    public boolean atTransfer(){
        return craneTransferReady;
    }
    public boolean Transfer(){
        switch (transferStage) {
            case 0: //retract for transfer plate position
                craneTransferReady = false;
                setExtendTargetPos(TRANSFER_ARM_LENGTH);
                transferTimer = futureTime(0.4);
                transferStage++;
                break;

            case 1: //send turret and shoulder to transfer position
                if((System.nanoTime() > transferTimer) && extensionOnTarget()){
                setShoulderTargetAngle(TRANSFER_SHOULDER_ANGLE);
                    robot.turret.articulate(Turret.Articulation.transfer);
                    transferTimer = futureTime(0.2);
                    transferStage++;
                }
                break;

            case 2: //short delay but the turret might not be transfer position when we return true
                if(System.nanoTime() > transferTimer) {
                    transferStage = 0;
                    craneTransferReady = true;
                    return true;
                }

        }
        return false;
    }

    public boolean oldTransfer(){
        switch (transferStage) {
            case 0: //conditions for gripper flipper to flip out
                craneTransferReady = false;
                robot.turret.articulate(Turret.Articulation.transfer);
                setShoulderTargetAngle(OLD_TRANSFER_SHOULDER_ANGLE);
                setExtendTargetPos(OLD_TRANSFER_ARM_LENGTH);
                transferTimer = futureTime(0.2);
                transferStage++;
                break;
            case 1:
                if(System.nanoTime() > transferTimer && robot.turret.atPosition()){
                    setShoulderTargetAngle(OLD_TRANSFER_SHOULDER_FLIPANGLE);
                    transferTimer = futureTime(0.4);
                    transferStage++;
                }
                break;
            case 2: //nudge flipper
                if(System.nanoTime() >= transferTimer) {
                    transferTimer = futureTime(1.5);
                    transferStage++;
                }
                break;

            case 3: //lower shoulder to transfer height
                if(System.nanoTime() >= transferTimer){
                    setShoulderTargetAngle(OLD_TRANSFER_SHOULDER_ANGLE);
                    transferStage = 0;
                    craneTransferReady = true;
                    return true;
                }
                break;

        }
        return false;
    }

    int postTransferStage = 0;
    long postTransferTimer = 0;
    boolean atPostTransfer = false;

    public static double tuneableTimer = 0.6;
    public boolean postTransfer(){ //flips the flipper gripper to the correct flipper gripper flipped position
        switch (postTransferStage){
            case 0:
                setShoulderTargetAngle(TRANSFER_SHOULDER_FLIPANGLE);
                atPostTransfer = false;
                flipToFlip();
                postTransferTimer = futureTime(tuneableTimer);
                postTransferStage++;
                break;
            case 1:
                if(System.nanoTime() > postTransferTimer){
                    flipToTension();
                    postTransferTimer = futureTime(0.32);
                    postTransferStage++;
                }
                break;
            case 2:
                if(System.nanoTime() > postTransferTimer){
                    grab();
                    postTransferTimer = futureTime(0.2);
                    postTransferStage++;
                }
                break;
            case 3:
                if(System.nanoTime() > postTransferTimer){
                    flipToHome();
                    postTransferStage++;
                }
                break;
            case 4:
                setShoulderTargetAngle(SAFE_SHOULDER_ANGLE);
                atPostTransfer = true;
                postTransferStage = 0;
                return true;
        }
        return false;
    }

    public boolean oldPostTransfer(){ //flips the flipper gripper to the correct flipper gripper flipped position
        switch (postTransferStage){
            case 0:
                atPostTransfer = false;
                setShoulderTargetAngle(0);
                postTransferStage++;
                break;
            case 1:
                if(shoulderOnTarget()){
                    setExtendTargetPos(0.2); //makes arm go fast forward
                    postTransferTimer = futureTime(0.2);
                    postTransferStage++;
                }
                break;
            case 2:
                if(System.nanoTime() >= postTransferTimer){
                    setExtendTargetPos(0.05); //snaps crane back making the flipper gripper flip to downwards flipper gripper flipping position
                    postTransferStage++;
                }
                break;
            case 3:
                atPostTransfer = true;
                postTransferStage = 0;
                return true;
        }
        return false;
    }

    public boolean atPostTransfer(){
        return atPostTransfer;
    }

    int coneStackStage = 0;
    public boolean coneStack(boolean rightConeStack){
        if(coneCycle(rightConeStack)){
            coneStackStage++;
        }
        return coneStackStage >= 1;
    }

    int coneCycleStage = 0;
    long cycleTimer = 0;
    boolean coneCycle(boolean rightConeStack){
        ConeStack obj = robot.field.getConeStack(rightConeStack);
        Pose2d pos = Field.convertToInches(obj.getPosition());
        switch(coneCycleStage) {
            case 0:
                if(goHome()){
                    coneCycleStage++;
                }
                break;
            case 1:
                if (goToFieldCoordinate(pos.getX()-1, pos.getY(), obj.z())) {
                    coneCycleStage++;
                    //nudgeLeft();
                    cycleTimer = futureTime(0.2);
                }
                break;
            case 2:
                if(System.nanoTime() >= cycleTimer && goToFieldCoordinate(pos.getX()-1, pos.getY(), obj.z() - 5)){
                    coneCycleStage++;
                    cycleTimer = futureTime(0.2);
                }
                break;
            case 3:
                if(System.nanoTime() >= cycleTimer) {
                    grab();
                    obj.takeCone();
                    cycleTimer = futureTime(0.1);
                    coneCycleStage++;
                }
                break;
            case 4:
                if(System.nanoTime() >= cycleTimer) {
                    setShoulderTargetAngle(getShoulderAngle() + 12);
                    coneCycleStage++;
                }
                break;
            case 5:
                if(shoulderOnTarget() && goHome()){
                    coneCycleStage++;
                }
                break;
            case 6:
                FieldThing temp;
                if (rightConeStack) {
                    temp = robot.field.objects[33];
                } else {
                    temp = robot.field.objects[32];
                }
                Pose2d tempPos = Field.convertToInches(temp.getPosition());
                if(rightConeStack) {
                    if (goToFieldCoordinate(tempPos.getX() + 2.5, tempPos.getY(), temp.z() + 3)) {
                        coneCycleStage++;
                        cycleTimer = futureTime(0.9);
                    }
                }else{
                    if (goToFieldCoordinate(tempPos.getX() + 1, tempPos.getY()-3.5, temp.z() + 3)) {
                        coneCycleStage++;
                        cycleTimer = futureTime(0.9);
                    }
                }
                break;
            case 7:
                if(System.nanoTime() >= cycleTimer) {
                    release();
                    cycleTimer = futureTime(0.4);
                    coneCycleStage++;
                }
                break;
            case 8:
                if(System.nanoTime() >= cycleTimer) {
                    setShoulderTargetAngle(getShoulderAngle() + 12);
                    coneCycleStage++;
                }
                break;
            case 9:
                if(shoulderOnTarget() && goHome()){
                    coneCycleStage++;
                }
                break;
            case 10:
                coneCycleStage = 0;
                return true;
        }
        return false;
    }

    public void holdTarget(double x, double y, double z){
        calculateFieldTargeting(x,y,z);
        goToCalculatedTarget();
    }

    public boolean goToFieldthing(FieldThing obj){
        Pose2d pos = Field.convertToInches(obj.getPosition());
        return goToFieldCoordinate(pos.getX(),pos.getY(),obj.z());
    }


    public boolean shoulderOnTarget(){
        return Math.abs(getShoulderTargetAngle()-getShoulderAngle()) < SHOULDER_ERROR_MAX;
    }

    public boolean extensionOnTarget(){
        return Math.abs(getExtenderTargetPos()-getExtendMeters()) < EXTENSION_ERROR_MAX;
    }

    public boolean turretOnTarget(){
        return Math.abs(robot.turret.getError()) < TURRET_ERROR_MAX;
    }


    int goTargetInd = 0;
    long goToTimer = 0;

    public boolean goToFieldCoordinate(double x, double y, double z){
        fieldPositionTarget = new Vector3(x,y,z);
        calculateFieldTargeting(fieldPositionTarget);
        switch(goTargetInd){
            case 0:
                extendNudgeStick();
                robot.turret.articulate(Turret.Articulation.runToAngle);
                setShoulderTargetAngle(calculatedAngle);
                setTargetTurretAngle(calculatedTurretAngle);
                goTargetInd++;
                break;
            case 1: //waits for shoulder and turret to finish moving to target then sets the extension to go to its target
                if(shoulderOnTarget() && turretOnTarget()){
                    setExtendTargetPos(calculatedLength);
                    setShoulderTargetAngle(calculatedAngle);
                    setTargetTurretAngle(calculatedTurretAngle);
                    goTargetInd++;
                }
                break;
            case 2: //checks if all are on target
                if(shoulderOnTarget() && extensionOnTarget() && turretOnTarget()){
                    goToTimer = futureTime(0.4);
                    goTargetInd++;
                }
                break;
            case 3:
                if(System.nanoTime() > goToTimer){
                    goTargetInd++;
                }
                break;
            case 4:
                goTargetInd = 0;
                return true;
        }

        return false;
    }

    public static Vector3 home = new Vector3(2, 0 ,12);

    int lock = 0;

    public boolean goLock(){
        switch (lock){
            case 0:
                setExtendTargetPos(SAFE_ARM_LENGTH);
                lock++;
                break;
            case 1:
                if(extensionOnTarget()){
                    setShoulderTargetAngle(HOME_SAFE_SHOULDER_ANGLE);
                    robot.turret.articulate(Turret.Articulation.lockTo180);
                    lock++;
                }
                break;
            case 2:
                lock = 0;
                return true;
        }
        return false;
    }
    int homeInd = 0;

    public boolean goHome(){
        fieldPositionTarget = new Vector3(home.x+robot.turret.getTurretPosition().getX(),home.y+robot.turret.getTurretPosition().getY(),home.z);
        calculateFieldTargeting(fieldPositionTarget);
        switch (homeInd){
            case 0:
                setExtendTargetPos(craneLengthOffset+0.1);
                tuckNudgeStick();
//                robot.underarm.articulate(UnderArm.Articulation.home);
                homeInd++;
                break;
            case 1:
                if(extensionOnTarget()) {
                    setShoulderTargetAngle(calculatedAngle);
                    setExtendTargetPos(calculatedLength);
                    homeInd++;
                }
                break;
            case 2:
                if(shoulderOnTarget() && extensionOnTarget()){
                    robot.articulate(Robot.Articulation.MANUAL);
                    robot.turret.articulate(Turret.Articulation.runToAngle);
                    robot.underarm.articulate(UnderArm.Articulation.manual);
                    homeInd = 0;
                    return true;
                }
        }
        return false;
    }

    FieldThing calcTurnPosition(FieldThing thing, double headingOffset){

        double angle = Math.toDegrees(Math.atan2(thing.y() - turretPos.getY(), thing.x()-turretPos.getX()));
        double bearing = Math.toRadians(angle + headingOffset);

        double x = getDistance()*Math.sin(bearing);
        double y = getDistance()*Math.cos(bearing);

        x += turretPos.getX();
        y += turretPos.getY();

        int z = thing.getHeight();

        return new FieldThing("temp" ,x,y,z);
    }


    int pickupConeStage = 0;
    long pickupTimer;
    FieldThing tempThing = null;

    boolean pickupCone(FieldThing obj) {
        switch (pickupConeStage) {
            case 0:
                grab();
                //set timer to allow bulb gripper enough time to change
                pickupTimer = futureTime(.5);
                pickupConeStage++;
                break;
            case 1:
                if(System.nanoTime() >= pickupTimer) {
                    setShoulderTargetAngle(getShoulderAngle() + 15);
                    pickupTimer = futureTime(.3);
                    pickupConeStage++;
                }
                break;
            case 2:
                //if(shoulderOnTarget()){
                if(System.nanoTime() >= pickupTimer && goHome()){
                    pickupConeStage++;
                }
                break;
            case 3:
                if(goToFieldthing(obj)){
                    pickupConeStage++;
                }
                break;
            case 4:
                pickupConeStage = 0;
                return true;
        }
        return false;
    }

    boolean pickupConeFromTransfer(FieldThing obj) {
        switch (pickupConeStage) {
            case 0:
                if(shoulderOnTarget()){
                    pickupConeStage++;
                }
                break;
            case 1:
                if(goToFieldthing(obj)){
                    pickupConeStage = 0;
                    return true;
                }
                break;
        }
        return false;
    }

    int dropConeStage = 0;
    long dropTimer;
    boolean dropCone(FieldThing obj) {
        switch (dropConeStage) {

            case 0:
                release();
                Vector3 dif = correctPos.subtract(fieldPositionTarget);
                //robot.driveTrain.setPoseEstimate(new Pose2d(robot.driveTrain.getPoseEstimate().getX()+dif.x,robot.driveTrain.getPoseEstimate().getY()+dif.y));
                dropTimer = futureTime(0.3); //enough time for cone to start dropping
                dropConeStage++;
                updateScoringPattern();
                break;
            case 1:
                if(System.nanoTime() >= dropTimer) {
                    //nudgeLeft();
                    setShoulderTargetAngle(getShoulderAngle() + 8);
                    dropConeStage++;
                }
                break;
            case 2:
                if(shoulderOnTarget() && goHome()){
                    dropConeStage++;
                }
                break;
            case 3:
                if(goToFieldthing(obj)){
                    dropConeStage++;
                }
                break;
            case 4:
                dropConeStage = 0;
                return true;
        }
        return false;
    }

    private int shoulderPosition = 0;
    private int shoulderDirectTickPos = 0;
    private int extendPosition = 0;
    double shoulderAngle = 0;
    double extendMeters = 0;
    double shoulderAmps, extenderAmps;

    public final double craneLengthOffset =  0.33;

    boolean inverseKinematic = false;
    double targetHeight = 0.0 ;
    double targetDistance = 0.3;
    double targetTurretAngle = 0;

    double angle;
    double length;

    double imuShoulderAngle;

    public static boolean antiTipping = true;
    public static boolean robotIsNotTipping = true;

    double turretPitch;

    Vector3 fieldPositionTarget;
    Vector3 robotPosition;

    @Override
    public void update(Canvas fieldOverlay) {

        calculateFieldTargeting(fieldPositionTarget);
        //nudgeDistance = nudgeDistanceSensor.getDistance(DistanceUnit.METER);

        robotPosition = new Vector3(robot.driveTrain.getPoseEstimate().getX(),robot.driveTrain.getPoseEstimate().getY(),shoulderHeight);

        //todo - switch shoulderPosition to read the dedicated angle encoder
        shoulderPosition = shoulderMotor.getCurrentPosition();
//        shoulderDirectTickPos = shoulderAngleEncoder.getCurrentPosition();
        extendPosition = extenderMotor.getCurrentPosition();

        //shoulderAngle = shoulderDirectTickPos / SHOULDER_DIRECT_TICKS_PER_DEGREE;
        extendMeters = extendPosition / EXTEND_TICKS_PER_METER;

        shoulderAmps = shoulderMotor.getCurrent(CurrentUnit.AMPS);
        extenderAmps = extenderMotor.getCurrent(CurrentUnit.AMPS);

        currentStateMachine.execute();


        if (SHOULDER_IMU_ENABLE) { //external shoulder imu is attached
            angles = shoulderImu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            gravity = shoulderImu.getGravity();
            shoulderAngle = -AngleUnit.DEGREES.normalize(AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.secondAngle));
        }
        else
            shoulderAngle = 0;

        turretAngles = turretImu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        turretGravity = turretImu.getGravity();

        turretPitch = -AngleUnit.DEGREES.normalize(AngleUnit.DEGREES.fromUnit(turretAngles.angleUnit, turretAngles.thirdAngle));

        if(antiTipping) {
            robotIsNotTipping = withinError(turretPitch, 0, 4); //checks if robot is happy
        }

        if(active) {
            articulate(articulation);
        }

        turretPos = robot.turret.getTurretPosition();
        axlePos = robot.turret.getAxlePosition();

        //update the turret's target
        robot.turret.setTargetHeading(targetTurretAngle);
        if(robotIsNotTipping) {

            if (shoulderActivePID)
                movePIDShoulder(SHOULDER_PID.kP, SHOULDER_PID.kI, SHOULDER_PID.kD, shoulderAngle, shoulderTargetAngle);
            else
                shoulderTargetAngle = shoulderAngle;

            if (extenderActivePID)
                movePIDExtend(EXTENDER_PID.kP, EXTENDER_PID.kI, EXTENDER_PID.kD, extendMeters, extenderTargetPos);
            else
                extenderTargetPos = extendMeters;
        }

        if(bulbGripped) {
            bulbServo.setPosition(servoNormalize(BULB_CLOSED_POS));
        }else {
            bulbServo.setPosition(servoNormalize(BULB_OPEN_POS));
        }

        updateNudgeStick();
        updateFlipperPosition();
    }

    Vector3 deltaGripperPosition = new Vector3(0,0,10);

    public void setSafeFieldTarget(){
        fieldPositionTarget = new Vector3(robot.driveTrain.getPoseEstimate().getX()+6,robot.driveTrain.getPoseEstimate().getY(),8);
    }

    public void adjustTurretAngle(double speed){
        if(robotIsNotTipping)targetTurretAngle = robot.turret.getHeading() + (TURRET_ADJUST * speed);
    }

    public void adjustDistance(double speed){
        if(robotIsNotTipping)setDistance(targetDistance + robot.deltaTime*(DISTANCE_ADJUST * speed));
    }

    public void adjustHeight(double speed){
        if(robotIsNotTipping)setHeight(targetHeight + robot.deltaTime*(HEIGHT_ADJUST * speed));
    }

    public void adjustExtend(double speed){
        if(robotIsNotTipping)setExtendTargetPos((getExtendMeters() + EXTEND_ADJUST * speed));
    }

    public void adjustShoulder(double distance){
        if(robotIsNotTipping)setShoulderTargetAngle((getShoulderAngle() + SHOULDER_ADJUST * distance));
    }

    public void adjustX(double speed){
        if(robotIsNotTipping){
            fieldPositionTarget.x += robot.deltaTime*(DISTANCE_ADJUST * speed);
        }
    }

    public void adjustY(double speed){
        if(robotIsNotTipping){
            fieldPositionTarget.y += robot.deltaTime*(DISTANCE_ADJUST * speed);
        }
    }

    public void adjustZ(double speed){
        if(robotIsNotTipping){
            fieldPositionTarget.z += robot.deltaTime*(HEIGHT_ADJUST * speed);
        }
    }



    public double getHeight(){
        return getExtendMeters()*Math.sin(Math.toRadians(getShoulderAngle()));
    }

    public double getDistance(){
        return getExtendMeters()*Math.cos(Math.toRadians(getShoulderAngle()));
    }
    public boolean calculateFieldTargeting(FieldThing obj){
        Pose2d coords = Field.convertToInches(obj.getPosition());
        fieldPositionTarget = new Vector3(coords.getX(),coords.getY(),obj.z()-1);
        calculateFieldTargeting(fieldPositionTarget);
        return true;
    }

    Pose2d turretPos = new Pose2d();
    Pose2d axlePos = new Pose2d();

    public static double shoulderHeight = 0.265;

    double calculatedTurretAngle;
    double calculatedHeight;
    double calculatedDistance;
    double calculatedAngle;
    double calculatedLength;

    public void setCraneTarget(double x, double y, double z){
        fieldPositionTarget = new Vector3(x,y,z);
    }

    public void enableShoulderPID(){
        shoulderActivePID = true;
    }

    public void disableShoulderPID(){
        shoulderActivePID = false;
    }

    public void enableExtensionPID(){
        extenderActivePID = true;
    }

    public void disableExtensionPID(){
        extenderActivePID = false;
    }

    public void enableAllPID() {
        enableShoulderPID();
        enableExtensionPID();
    }

    public void disableAllPID(){
        disableExtensionPID();
        disableShoulderPID();
    }

    public boolean calculateFieldTargeting(Vector3 vec){
        return calculateFieldTargeting(vec.x,vec.y,vec.z);
    }

    public static double SHOULDER_BEND_CORRECTION = 0.1;

    public boolean calculateFieldTargeting(double x, double y, double z){ //THIS IS IN INCHES!!!!!!!!

        z /= INCHES_PER_METER;

        calculatedTurretAngle = Math.toDegrees(Math.atan2(y - turretPos.getY(), x-turretPos.getX()));

        calculatedHeight = z-shoulderHeight;

        calculatedDistance = (Math.sqrt(Math.pow(y - axlePos.getY(),2) + Math.pow(x - axlePos.getX(),2)))/INCHES_PER_METER;

        calculatedHeight += SHOULDER_BEND_CORRECTION*calculatedDistance;

        calculatedAngle = Math.toDegrees(Math.atan2(calculatedHeight, calculatedDistance));
        calculatedLength = (Math.sqrt(Math.pow(calculatedHeight, 2) + Math.pow(calculatedDistance, 2)));
////todo Vance? this earns a null object exception
//        if(robot.checkCollision(calculatedAngle,calculatedTurretAngle)){
//            calculatedAngle = getShoulderAngle();
//            calculatedLength = getExtendMeters();
//            robot.underarm.articulate(UnderArm.Articulation.safe);
//        }

        return true;
    }

    public void goToCalculatedTarget(){
        setTargetTurretAngle(calculatedTurretAngle);
        targetHeight = calculatedHeight;
        targetDistance = calculatedDistance;
        setShoulderTargetAngle(calculatedAngle);
        setExtendTargetPos(calculatedLength);
    }

    double orbit_time = 0.0;
    public static double orbit_speed = 1.0;
    public static double orbit_radius = 8.0;
    public static double orbit_distance = 30;
    public static double orbit_height = 25;

    public void doOrbit(){

        Pose2d robotPos = robot.driveTrain.getPoseEstimate();
        calculateFieldTargeting(orbit_distance + orbit_radius*Math.sin(orbit_time),-35 + orbit_radius*Math.cos(orbit_time),orbit_height);

        orbit_time += orbit_speed*robot.deltaTime;
    }

    public void setTargetTurretAngle(double target){
        targetTurretAngle = target;
    }
    public boolean setHeight(double t){
        targetHeight = Math.min(1.8,Math.max(t, -0.2));
        return true;
    }

    public boolean setDistance(double t){
        targetDistance = Math.min(1.8,Math.max(t, 0));
        return true;
    }

    public boolean goToTarget(){
        return shoulderPID.onTarget() && extendPID.onTarget() && robot.turret.isTurretNearTarget();
    }
    boolean pickUpConeInitialized = false;
    double pickUpLastTime;
    public boolean descendToCone(){
        if(!pickUpConeInitialized){
            pickUpLastTime = System.nanoTime() / 1e9;
            pickUpConeInitialized = true;
            return false;
        }

        if(shoulderMotor.getCurrent(CurrentUnit.AMPS) > 1){
            pickUpConeInitialized = false;
            return true;
        }
        targetHeight = targetHeight - (System.nanoTime() / 1e9 - pickUpLastTime) * PICK_UP_VELOCITY;
        return false;
    }

    StateMachine pickUpConeStateMachine = Utils.getStateMachine(new Stage())
            .addState(() -> descendToCone())
            .addState(() -> {
                grab(); return true;})
            .addTimedState(1, () -> {}, ()-> {})
            .addState(() -> setHeight(targetHeight + HEIGHT_AFTER_PICKING_UP_CONE) )
            .build();

    StateMachine dropConeStateMachine = Utils.getStateMachine(new Stage())
            .addState(() -> descendToCone())
            .addState(() -> setHeight(targetHeight + 1))
            .addState(() -> goToTarget())
            .addState(() -> {
                release(); return true;})
            .addTimedState((float)0.5, () -> {}, ()-> {})
            .addState(() -> setHeight(targetHeight + HEIGHT_AFTER_PICKING_UP_CONE) )
            .build();

    public void setCurrentStateMachineToPickUp(){
        currentStateMachine = pickUpConeStateMachine;
    }
    public void setCurrentStateMachineToDropCone(){
        currentStateMachine = dropConeStateMachine;
    }
    public void setCurrentStateMachine(StateMachine statemachine){
        currentStateMachine = statemachine;
    }

    @Override
    public void stop() {
        setShoulderPwr(0,0);
        setExtenderPwr(0,0);
        setShoulderActivePID(false);
        setextenderActivePID(false);
    }

    public void toggleGripper(){
        bulbGripped = !bulbGripped;
    }

    public void grab(){
        bulbGripped = true;
    }

    public void pickupSequence(){
        articulate(Articulation.scoreCone);
    }

    public void release(){
        bulbGripped = false;
    }

    public boolean calibrateEnabled(){
       return craneCalibrationEnabled;
    }

    Vector3 correctPos = new Vector3(0,0,0);
    public void dropSequence(){
        articulate(Articulation.dropCone);
        correctPos = new Vector3(robot.field.getPatternObject().x(),robot.field.getPatternObject().y(),robot.field.getPatternObject().z());
    }

    public void setGripper(boolean g){
        bulbGripped = g;
    }

    @Override
    public String getTelemetryName() {
        return "Crane";
    }

    public void setExtenderPwr(double pwrMin, double pwrMax){ extendPID.setOutputRange(pwrMin,pwrMax); }
    public void setextenderActivePID(boolean isActive){extenderActivePID = isActive;}
    public void setShoulderActivePID(boolean isActive){shoulderActivePID = isActive;}


    public void setShoulderPwr(double pwrMin, double pwrMax){ shoulderPID.setOutputRange(pwrMin,pwrMax); }
    public  void setShoulderTargetAngle(double t){ shoulderTargetAngle = (Math.max(Math.min(t,SHOULDER_DEG_MAX),SHOULDER_DEG_MIN)); }
    public  double getShoulderTargetAngle(){ return shoulderTargetAngle; }
    public double getExtenderTargetPos(){ return extenderTargetPos+craneLengthOffset; }
    public  void setExtendTargetPos(double t){ extenderTargetPos = Math.min(EXTENDER_TICS_MAX/EXTEND_TICKS_PER_METER,Math.max(t-craneLengthOffset, 0)); }
    public boolean nearTargetShoulder(){
        if ((Math.abs( getShoulderAngle()- getShoulderTargetAngle()))<2) return true;
        else return false;
    }

    public double getShoulderAngle(){ return shoulderAngle;}
    public double getExtendMeters(){return extendMeters+craneLengthOffset;}
    public double getExtendInches(){return getExtendMeters() * INCHES_PER_METER; }

    @Override
    public Map<String, Object> getTelemetry(boolean debug) {
        Map<String, Object> telemetryMap = new LinkedHashMap<>();

        telemetryMap.put("Current Articulation", articulation);
        telemetryMap.put("Transfer Stage", transferStage);
        telemetryMap.put("Post Transfer Stage", postTransferStage);
        telemetryMap.put("Pickup Stage", pickupConeStage);
        telemetryMap.put("Drop Stage", dropConeStage);
        telemetryMap.put("ConeStackStage", coneStackStage);
        telemetryMap.put("ConeCycle Stage", coneCycleStage);
        telemetryMap.put("GoTo Stage", goTargetInd);
        telemetryMap.put("Home Stage", homeInd);
        telemetryMap.put("Shoulder On Target", shoulderOnTarget());
        telemetryMap.put("Extend On Target", extensionOnTarget());
        telemetryMap.put("Turret On Target", turretOnTarget());
        telemetryMap.put("Shoulder Error", shoulderPID.getError());
        telemetryMap.put("Extend Error", extendPID.getError());

        if (debug) {
            telemetryMap.put("Shoulder On Target", shoulderOnTarget());
            telemetryMap.put("Extension On Target", extensionOnTarget());
            telemetryMap.put("Turret On Target", turretOnTarget());
            telemetryMap.put("Shoulder Power", shoulderMotor.getPower());
            telemetryMap.put("Shoulder Current", shoulderMotor.getCurrent(CurrentUnit.AMPS));
            telemetryMap.put("Extend Error", extendPID.getError());
            telemetryMap.put("Turret Pitch", turretPitch);
            telemetryMap.put("Robot X", robot.driveTrain.getPoseEstimate().getX());
            telemetryMap.put("Robot Y", robot.driveTrain.getPoseEstimate().getY());
            telemetryMap.put("Turret X", turretPos.getX());
            telemetryMap.put("Turret Y", turretPos.getY());
            telemetryMap.put("Axle X", axlePos.getX());
            telemetryMap.put("Axle Y", axlePos.getY());
            telemetryMap.put("Target Distance", targetDistance);
            telemetryMap.put("Target Height", targetHeight);
            telemetryMap.put("Target Turret Angle", targetTurretAngle);
            telemetryMap.put("Target Angle", angle);
            telemetryMap.put("Target Length", length);
            telemetryMap.put("IK ON", inverseKinematic);
            telemetryMap.put("Calculated Turret Angle", calculatedTurretAngle);
            telemetryMap.put("Calculated Height", calculatedHeight);
            telemetryMap.put("Calculated Distance", calculatedDistance);
            telemetryMap.put("Calculated Angle", calculatedAngle);
            telemetryMap.put("Calculated Length", calculatedLength);
            telemetryMap.put("Field Target X", fieldPositionTarget.x);
            telemetryMap.put("Field Target Y", fieldPositionTarget.y);
            telemetryMap.put("Field Target Z", fieldPositionTarget.z);
            telemetryMap.put("Distance", getDistance());
            telemetryMap.put("Height", getHeight());
            telemetryMap.put("Calibrate Stage", calibrateStage);
            telemetryMap.put("Bulb Pos", bulbGripped);
            telemetryMap.put("Bulb Pos Ticks", servoDenormalize(bulbServo.getPosition()));
            telemetryMap.put("Extend Meters", extendMeters);
            telemetryMap.put("Extend Tics", extendPosition);
            telemetryMap.put("Extend Amps", extenderAmps);
            telemetryMap.put("Extend Power", extenderMotor.getPower());
            telemetryMap.put("Extend Active PID", extenderActivePID);
            telemetryMap.put("Extend Target", extenderTargetPos);
            telemetryMap.put("Extend PID", extendCorrection);
            telemetryMap.put("Extend Run Amp", runExtendAmp);
            telemetryMap.put("Extend Max Tics", extendMaxTics);

            telemetryMap.put("IMU Shoulder Angle", imuShoulderAngle);
            telemetryMap.put("Shoulder Angle", shoulderAngle);
            telemetryMap.put("Shoulder Tics", shoulderPosition);
            telemetryMap.put("Shoulder Power", shoulderMotor.getPower());
            telemetryMap.put("Shoulder Amps", shoulderAmps);
            telemetryMap.put("Shoulder Direct Angle Tics", shoulderDirectTickPos);
            telemetryMap.put("Shoulder Target", shoulderTargetAngle);
            telemetryMap.put("Shoulder Active", shoulderActivePID);
            telemetryMap.put("Shoulder PID Output", shoulderCorrection);
            telemetryMap.put("Running Amp", runShoulderAmp);
            telemetryMap.put("Nudge Distance Sensor", nudgeDistance);
            telemetryMap.put("Nudge Target", nudgeStickServo.getPosition());
            telemetryMap.put("Nudge Target Calc", nudgeTarget);
            telemetryMap.put("Nudge Target Ticks?", servoDenormalize(nudgeStickServo.getPosition()));
            telemetryMap.put("Nudge Tucked?", nudgeTuck);
            telemetryMap.put("Flipper Target", flipperPos);
            telemetryMap.put("Flipper Target Ticks", servoDenormalize(flipperServo.getPosition()));


        }else{

        }
        return telemetryMap;
    }

}
