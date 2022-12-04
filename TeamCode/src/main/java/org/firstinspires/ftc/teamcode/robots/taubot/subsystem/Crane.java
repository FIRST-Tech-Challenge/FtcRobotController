package org.firstinspires.ftc.teamcode.robots.taubot.subsystem;

import static org.firstinspires.ftc.teamcode.robots.taubot.util.Utils.servoNormalize;
import static org.firstinspires.ftc.teamcode.robots.taubot.util.Constants.INCHES_PER_METER;
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
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.teamcode.robots.taubot.FieldObject;
import org.firstinspires.ftc.teamcode.robots.taubot.simulation.DcMotorExSim;

import org.firstinspires.ftc.teamcode.robots.taubot.simulation.ServoSim;

import org.firstinspires.ftc.teamcode.robots.taubot.util.CranePositionMemory;
import org.firstinspires.ftc.teamcode.robots.taubot.util.Utils;
import org.firstinspires.ftc.teamcode.statemachine.Stage;
import org.firstinspires.ftc.teamcode.statemachine.StateMachine;
import org.firstinspires.ftc.teamcode.util.PIDController;

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
    public static double SHOULDER_DEG_MAX = 90-7.25; //max elevation of shoulder when stalled up - measured by inclinometer
    public static double SHOULDER_DIRECT_TICKS_PER_DEGREE = (1937-88)/(SHOULDER_DEG_MAX); //todo verify/update when sensor secured and robot is more burned in - before tuning precision articulation

    public static double SHOULDER_TICK_MAX = 1849;

    public static double EXTEND_TICKS_PER_METER = 806/.2921; //todo verify this is still true

    public static double kF = 0.0;

    public static PIDCoefficients SHOULDER_PID = new PIDCoefficients(0.03, 0.001, 0.006);
    public static double SHOULDER_MAX_PID_OUTPUT = 1;
    public static double SHOULDER_MIN_PID_OUTPUT = -1;
    public static double SHOULDER_TOLERANCE = 1;
    public static double SHOULDER_POWER = 1.0;
    public static double SHOULDER_ADJUST = 13;
    public static double EXTEND_ADJUST = .05;
    public static double TURRET_ADJUST = 20;

    public static double kE = 0.0;
    public static PIDCoefficients EXTENDER_PID = new PIDCoefficients(25, 0, 0.005);
    public static double EXTEND_MAX_PID_OUTPUT = 0.8;
    public static double EXTEND_MIN_PID_OUTPUT = -0.8;
    public static double EXTENDER_TOLERANCE = 1;
    public static double EXTENDER_POWER = 1.0;
    public static double EXTENDER_TICS_MIN = 0;
    public static double EXTENDER_TICS_MAX = 3100; // of the robot
    boolean EXTENDER_CALIBRATE_MAX = false; //keep false except if calibrating EXTENDER_TICS_MAX

    public static double BULB_OPEN_POS = 1500;
    public static double BULB_CLOSED_POS = 1750;

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
    public DcMotorEx extenderMotor;
    public DcMotorEx shoulderMotor;
    public DcMotorEx turretMotor;
    public DcMotor shoulderAngleEncoder;

    private PIDController shoulderPID;
    private PIDController extendPID;

    private boolean bulbGripped;

    private Robot robot;


    private Articulation articulation = Articulation.manual;

    boolean USE_MOTOR_SMOOTHING = true;

    private Servo nudgeStickServo;

    BNO055IMU imu;

    Orientation angles;
    Acceleration gravity;

    StateMachine currentStateMachine = Utils.getStateMachine(new Stage()).addState(()->{return true;}).build();

    public Crane(HardwareMap hardwareMap, Robot robot, boolean simulated) {
        this.robot = robot;
        extenderTargetPos = 0;
        shoulderTargetAngle = 0;
        if (simulated) {
            shoulderMotor = new DcMotorExSim(USE_MOTOR_SMOOTHING);
            extenderMotor = new DcMotorExSim(USE_MOTOR_SMOOTHING);
            turretMotor = new DcMotorExSim(USE_MOTOR_SMOOTHING);
            shoulderAngleEncoder = new DcMotorExSim(USE_MOTOR_SMOOTHING);
            bulbServo = new ServoSim();
            nudgeStickServo = new ServoSim();
        } else {
            shoulderMotor = hardwareMap.get(DcMotorEx.class, "shoulder");
            shoulderAngleEncoder = hardwareMap.get(DcMotorEx.class, "shoulderAngleEncoder"); //just a REV shaft encoder - no actual motor
            extenderMotor = hardwareMap.get(DcMotorEx.class, "extender");
            turretMotor = hardwareMap.get(DcMotorEx.class, "turret");
            shoulderMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            extenderMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            extenderMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            extenderMotor.setTargetPosition(0);
            shoulderMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            extenderMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            shoulderMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            bulbServo = hardwareMap.get(Servo.class, "servoGripper");
            nudgeStickServo = hardwareMap.get(Servo.class, "nudgeSwivel");

            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
            parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
            parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
            parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
            parameters.loggingEnabled      = true;
            parameters.loggingTag          = "IMU";
            parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

            imu = hardwareMap.get(BNO055IMU.class, "shoulderIMU");
            imu.initialize(parameters);

            imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);


        }
        extendPID = new PIDController(0,0,0);
        extendPID.setOutputRange(EXTEND_MIN_PID_OUTPUT, EXTEND_MAX_PID_OUTPUT);
        shoulderPID = new PIDController(0,0,0);
        shoulderPID.setOutputRange(SHOULDER_MIN_PID_OUTPUT,SHOULDER_MAX_PID_OUTPUT);
        shoulderPID.setIntegralCutIn(40);
        shoulderPID.enableIntegralZeroCrossingReset(false);

        shoulderTargetAngle = 5;
    }

    int calibrateStage=0;
    double futureTime;
    double runShoulderAmp;
    double runExtendAmp;
    int extendMaxTics;

    public boolean calibrate(){
        // to calibrate we want the arm to be fully retracted and the shoulder
        // to be fully up at the physical stop as a repeatable starting position
        switch (calibrateStage) {
            case 0:
                calibrated = false; //allows us to call calibration mid-match in an emergency
                //operator instruction: physically push arm to about 45 degrees and extend by 1 slide before calibrating
                //shoulder all the way up and retract arm until they safely stall
                shoulderActivePID = false;
                extenderActivePID = false;
                shoulderMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                extenderMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                shoulderMotor.setPower(.2); //move up at low power
                extenderMotor.setPower(-0.2);
                futureTime = futureTime(.25);
                calibrateStage++;
                break;

            case 1:
                if(System.nanoTime() > futureTime){
                    //sample low load amps
                    runShoulderAmp = shoulderMotor.getCurrent(CurrentUnit.AMPS);
                    runExtendAmp = extenderMotor.getCurrent(CurrentUnit.AMPS);
                    calibrateStage++;
                }
                break;

            case 2:
                // both motors are stalled
                if(shoulderMotor.getCurrent(CurrentUnit.AMPS) > 1 && extenderMotor.getCurrent(CurrentUnit.AMPS) > 1){
                    extenderMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    extenderMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    extenderMotor.setPower(-0.1); //barely stop it from extending
                    //enable PID on shoulder and rotate down to horizontal
                    shoulderMotor.setPower(0.0);
                    shoulderMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    shoulderMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    shoulderAngleEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    shoulderTargetAngle = -SHOULDER_DEG_MAX;
                    shoulderActivePID = true;
                    calibrateStage++;
                }
                break;

            case 3:
                if (shoulderAngle > shoulderTargetAngle -2 && shoulderAngle < shoulderTargetAngle +2){ //shoulder is horizontal, so reset encoder to begin from here - normally use shoulderPID.onTarget(), but that might not be setup correctly yet
                    shoulderActivePID = false;
                    shoulderAngleEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); //now horizontal should be at zero
                    shoulderTargetAngle =0;
                    shoulderActivePID = true;
                    // WATCH OUT - begin extending arm - we want to find max extension (once established we will normally not do this)
                    if (EXTENDER_CALIBRATE_MAX) extenderMotor.setPower(.6);
                    calibrateStage++;
                }
                break;

            case 4:
                if (EXTENDER_CALIBRATE_MAX) {
                    if (extenderMotor.getCurrent(CurrentUnit.AMPS) > 6) { //should be stalled at full extension
                        extendMaxTics = extenderMotor.getCurrentPosition();
                        calibrateStage++;
                    }
                }else {
                    extendMaxTics = 3075;
                    calibrateStage++;
                }
                break;

            case 5: //enable extender PID to zero position - give 2 seconds to retract a good bit
                extenderMotor.setPower(0.0);
                extenderTargetPos = 0;
                extenderActivePID = true;
                futureTime = futureTime(0.5);
                calibrateStage++;
                break;

            case 6:
                if (System.nanoTime()>futureTime) {
                    calibrateStage = 0;
                    calibrated = true;
                    shoulderTargetAngle = 5; //initial angle up to clear look at signal
                    return true;
                }
                break;

        }
        return false;
    }
    boolean calibrated = true;

    double shoulderCorrection = 0;
    double shoulderPwr = 1;
    double shoulderTargetAngle = 0;

    public static double NUDGE_CENTER_LEFT = 1950;
    public static double NUDGE_CENTER_RIGHT = 2020;
    public static double NUDGE_LEFT_POS = 1000;  //home position - stowed up
    public static double NUDGE_RIGHT_POS = 2400;

    //keeps track of current nudge position
    public static int nudgeIndex = 1;

    public void nudgeCenter(boolean approachingClockwise){
        if (approachingClockwise) {
            nudgeStickServo.setPosition(servoNormalize(NUDGE_CENTER_LEFT));
            nudgeIndex = 1;
        }else {
            nudgeStickServo.setPosition(servoNormalize(NUDGE_CENTER_RIGHT));
            nudgeIndex = 2;
        }
    }
    public void nudgeLeft(){
        nudgeStickServo.setPosition(servoNormalize(NUDGE_LEFT_POS));
        nudgeIndex = 0;
    }
    public void nudgeRight(){
        nudgeStickServo.setPosition(servoNormalize(NUDGE_RIGHT_POS));
        nudgeIndex = 3;
    }

    public void incNudgeIndex(){
        if(nudgeIndex < 3){
            nudgeIndex++;
        }
        updateNudgeStick();
    }
    public void decNudgeIndex(){
        if(nudgeIndex > 0){
            nudgeIndex--;
        }
        updateNudgeStick();
    }

    public void updateNudgeStick(){
        switch (nudgeIndex){
            case 0:
                nudgeLeft();
                break;
            case 1:
                nudgeCenter(true);
                break;
            case 2:
                nudgeCenter(false);
                break;
            case 3:
                nudgeRight();
                break;
            default:
                nudgeCenter(true);
                break;
        }
    }

    public void movePIDShoulder(double Kp, double Ki, double Kd, double currentTicks, double targetTicks) {

        //initialization of the PID calculator's output range, target value and multipliers
        shoulderPID.setOutputRange(SHOULDER_MIN_PID_OUTPUT, SHOULDER_MAX_PID_OUTPUT);
        shoulderPID.setPID(Kp, Ki, Kd);
        shoulderPID.setSetpoint(targetTicks);
        shoulderPID.enable();

        //initialization of the PID calculator's input range and current value
        shoulderPID.setInput(currentTicks);

        //calculates the correction to apply
        shoulderCorrection = shoulderPID.performPID();

        //moves elbow with the correction applied
        shoulderMotor.setPower(shoulderCorrection);
    }

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
        defaultPosition,
        dropCone,
        pickupCone
    }

    public Articulation articulate(Articulation target){
        articulation = target;

        switch(articulation){
            case defaultPosition:
                setShoulderTargetAngle(SHOULDER_DEG_MAX);
                robot.turret.setTargetHeading(0);
                setExtendTargetPos(0.2);
                break;
            case manual:

                break;
            case dropCone:
                if(dropCone()){
                    articulation = Articulation.manual;
                    return Articulation.manual;
                }

                break;
            case pickupCone:
                if(pickupCone()){
                    articulation = Articulation.manual;
                    return Articulation.manual;
                }

                break;
            default:
                return target;
        }
        return target;
    }

    int pickupConeStage = 0;
    long pickupTimer;
    boolean pickupCone() {
        switch (pickupConeStage) {
            case 0:
                //save values for next pickup
                grab();

                //set timer to allow bulb gripper enough time to change
                pickupTimer = futureTime(.5);
                pickupConeStage++;

                break;

            case 1: //initial cone lift

                if(System.nanoTime() > pickupTimer) //waiting until gripped, then start lifting:
                {
                    setShoulderTargetAngle(getShoulderAngle()+15); //todo - this should be a vertical offset (not this angular shortcut)
                    //enough  time for the cone to lift a cone off of a stack so it doesn't drag the stack down when retracting
                    if(robot.turret.distanceBetweenAngles(robot.turret.getHeading(),drop.getHeadingMemory()) < 0){
                        nudgeCenter(true);
                    }else{
                        nudgeCenter(false);
                    }
                    pickupTimer = futureTime(.5);
                    pickupConeStage++;

                }
                break;

            case 2: //waiting on initial lift, then move arm to defaultpos - this is usually a retraction and a lift
                if(System.nanoTime() > pickupTimer)
                {
                    setShoulderTargetAngle(defaultPos.getShoulderMemory());
                    setExtendTargetPos(defaultPos.getExtendMemory());
                    pickupTimer = futureTime(1.5); //typical time to retract enough to start turntable
                    pickupConeStage++;
                }
                break;

            case 3: //move to previously set drop position

                //todo - this is currently time based and it should maybe be based on lift achieved (if achievable - long extensions can't actually lift due to torque needed)
                if(System.nanoTime() > pickupTimer) {
                    //move to previous drop location with a little extra height
                    //robot.turret.setTargetHeading(drop.getHeadingMemory());
                    targetTurretAngle = drop.getHeadingMemory();
                    setShoulderTargetAngle(drop.getShoulderMemory());
                    pickupTimer = futureTime(0.7);
                    pickupConeStage++;
                }
                break;
            case 4:
                if(System.nanoTime() > pickupTimer){
                    setExtendTargetPos(drop.getExtendMemory());
                    pickupConeStage = 0;
                    return true;
                }
            default:
                return false;
        }
        return false;
    }

    int dropConeStage = 0;
    long dropTimer;
    boolean dropCone() {
        switch (dropConeStage) {

            case 0:
                release();
                dropTimer = futureTime(0.3); //enough time for cone to start dropping
                dropConeStage++;

            case 1:
                if(System.nanoTime() > dropTimer){
                    if(nudgeIndex == 2) {
                        nudgeRight();
                    }else if(nudgeIndex == 1){
                        nudgeLeft();
                    }
                    dropTimer = futureTime(0.5); //time for nudge right to complete
                    dropConeStage++;
                }
                break;
            case 2: //lift a little to clear junction

                if(System.nanoTime() > dropTimer) //waiting until released, then start lifting:
                {
                    setShoulderTargetAngle(getShoulderAngle()+20); //todo - this should be a vertical offset (not this angular shortcut)
                    //enough  time for the cone to lift a cone off of a stack so it doesn't drag the stack down when retracting
                    dropTimer = futureTime(.3);
                    dropConeStage++;

                }
                break;

            case 3: //waiting on lift, then move arm to defaultpos - this is usually a retraction and a lift
                if(System.nanoTime() > dropTimer)
                {
                    nudgeLeft();
                    setExtendTargetPos(defaultPos.getExtendMemory());
                    dropTimer = futureTime(0.7); //typical time to retract enough to start turntable
                    dropConeStage++;
                }
                break;
            case 4:
                if(System.nanoTime() > dropTimer){
                    setShoulderTargetAngle(defaultPos.getShoulderMemory());
                    dropTimer = futureTime(0.7);
                    dropConeStage++;
                }
                break;

            case 5: //move to previously set pickup position

                //todo - this is currently time based and it should maybe be based on lift achieved (if achievable - long extensions can't actually lift due to torque needed)
                if(System.nanoTime() > dropTimer) {
                    targetTurretAngle = pickup.getHeadingMemory();
                    //move to previous drop location with a little extra height
                    //robot.turret.setTargetHeading(pickup.getHeadingMemory()); //doesn't work cause - gets overriden
                    dropConeStage++;
                    pickupTimer = futureTime(0.7);
                }
                break;
            case 6:
                if(System.nanoTime() > dropTimer){
                    setShoulderTargetAngle(pickup.getShoulderMemory()); //return high
                    dropConeStage++;
                    pickupTimer = futureTime(0.7);
                }
                break;
            case 7:
                if(System.nanoTime() > pickupTimer){
                    setExtendTargetPos(pickup.getExtendMemory());
                    dropConeStage = 0;
                    return true;
                }
            default:
                return false;
        }
        return false;
    }

    private int shoulderPosition = 0;
    private int shoulderDirectTickPos = 0;
    private int extendPosition = 0;
    double shoulderAngle = 0;
    double extendMeters = 0;
    double shoulderAmps, extenderAmps;

    private double craneLengthOffset =  0.432;



    boolean inverseKinematic = false;
    double targetHeight = 20;
    double targetDistance = 20;
    double targetTurretAngle = 0;

    double angle;
    double length;

    double imuShoulderAngle;

    @Override
    public void update(Canvas fieldOverlay) {
        //todo - switch shoulderPosition to read the dedicated angle encoder
        shoulderPosition = shoulderMotor.getCurrentPosition();
        shoulderDirectTickPos = shoulderAngleEncoder.getCurrentPosition();
        extendPosition = extenderMotor.getCurrentPosition();

        //shoulderAngle = shoulderDirectTickPos / SHOULDER_DIRECT_TICKS_PER_DEGREE;
        extendMeters = extendPosition / EXTEND_TICKS_PER_METER;

        shoulderAmps = shoulderMotor.getCurrent(CurrentUnit.AMPS);
        extenderAmps = extenderMotor.getCurrent(CurrentUnit.AMPS);

        currentStateMachine.execute();

        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        gravity  = imu.getGravity();

        shoulderAngle = -AngleUnit.DEGREES.normalize(AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.secondAngle));

        if(calibrated) {
            //run the current articulation
            articulate(articulation);

        }
        //update the turret's target
        robot.turret.setTargetHeading(targetTurretAngle);
        if(shoulderActivePID)
            movePIDShoulder(SHOULDER_PID.kP, SHOULDER_PID.kI, SHOULDER_PID.kD, shoulderAngle, shoulderTargetAngle);
        else
            shoulderTargetAngle = shoulderAngle;

        if(extenderActivePID)
            movePIDExtend(EXTENDER_PID.kP, EXTENDER_PID.kI, EXTENDER_PID.kD, extendMeters, extenderTargetPos);
        else
            extenderTargetPos = extendMeters;

        if(bulbGripped) {
            bulbServo.setPosition(servoNormalize(BULB_CLOSED_POS));
        }else {
            bulbServo.setPosition(servoNormalize(BULB_OPEN_POS));
        }
    }

    public void adjustTurretAngle(double speed){
        targetTurretAngle = robot.turret.getHeading() + (TURRET_ADJUST * speed);
    }

    public void adjustDistance(double speed){
        setDistance(getDistance() + (0.1 * speed));
    }

    public void adjustHeight(double speed){
        setHeight(getHeight() + (0.1 * speed));
    }

    public void adjustExtend(double speed){
        setExtendTargetPos((getExtendMeters() + EXTEND_ADJUST * speed));
    }

    public void adjustShoulder(double distance){
        setShoulderTargetAngle((getShoulderAngle() + SHOULDER_ADJUST * distance));
    }

    public double getHeight(){
        return INCHES_PER_METER * getExtendMeters()*Math.sin(Math.toRadians(getShoulderAngle()));
    }

    public double getDistance(){
        return INCHES_PER_METER * getExtendMeters()*Math.cos(Math.toRadians(getShoulderAngle()));
    }
    public boolean setTargets(FieldObject obj){
        return setTargets(obj.x(),obj.y(),obj.z());
    }

    Pose2d turretPos = new Pose2d();
    Pose2d axlePos = new Pose2d();

    public static double axleOffset = -9;
    public static double shoulderHeight = 0.14;

    public boolean setTargets(double x, double y, double z){

        z /= INCHES_PER_METER;

        Pose2d robotPos = robot.driveTrain.getPoseEstimate();
        turretPos = robot.turret.getTurretPosition(robotPos);
        targetTurretAngle = Math.toDegrees(Math.atan2(y - turretPos.getY(), x-turretPos.getX()));

        axlePos = new Pose2d(turretPos.getX()+axleOffset*Math.cos(Math.toRadians(robot.turret.getHeading())), turretPos.getY()+axleOffset*Math.sin(Math.toRadians(robot.turret.getHeading())));

        targetHeight = z-shoulderHeight;

        targetDistance = (Math.sqrt(Math.pow(y - axlePos.getY(),2) + Math.pow(x - axlePos.getX(),2)))/INCHES_PER_METER;

        angle = Math.toDegrees(Math.atan(targetHeight / targetDistance));
        length = (Math.sqrt(Math.pow(targetHeight, 2) + Math.pow(targetDistance, 2)) - craneLengthOffset);
        setShoulderTargetDeg(angle);
        setExtendTargetDistance(length);
        robot.turret.setTargetHeading(targetTurretAngle);

        return true;
    }

    public void setTargetTurretAngle(double target){
        targetTurretAngle = target;
    }
    public boolean setHeight(double newHeight){
        targetHeight = newHeight;
        return true;
    }

    public boolean setDistance(double newDistance){
        targetDistance = newDistance;
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

        if(bulbGripped == false){
            recordPickup();
        }
        bulbGripped = true;
    }

    CranePositionMemory pickup = new CranePositionMemory(0,20,0.2);
    CranePositionMemory defaultPos = new CranePositionMemory(0,85,0);
    CranePositionMemory drop = new CranePositionMemory(0,45,0.2);

    private void recordPickup(){
        pickup.setCranePositionMemory(robot.turret.getHeading(), shoulderAngle+20,extendMeters);
    }

    public void pickupSequence(){
        articulate(Articulation.pickupCone);
    }

    public void release(){

        if(bulbGripped == true){
            recordDrop();
        }
        bulbGripped = false;
    }

    private void recordDrop(){
        drop.setCranePositionMemory(robot.turret.getHeading(), shoulderAngle,extendMeters);
    }

    public void enableCalibrate(){
        craneCalibrationEnabled = true;
    }
    public void setCalibrated(){
        calibrated = true;
    }

    public boolean calibrateEnabled(){
       return craneCalibrationEnabled;
    }

    public void dropSequence(){
        articulate(Articulation.dropCone);
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
    public void setShoulderTargetDeg(double deg){
        shoulderTargetAngle = deg;
    }
    public void setExtendTargetDistance(double dis){
        setExtendTargetPos(dis);
    }
    public void setShoulderPwr(double pwrMin, double pwrMax){ shoulderPID.setOutputRange(pwrMin,pwrMax); }
    public  void setShoulderTargetAngle(double t){ shoulderTargetAngle = (Math.max(Math.min(t,SHOULDER_TICK_MAX/SHOULDER_DIRECT_TICKS_PER_DEGREE),-10)); }
    public  double getShoulderTargetAngle(){ return shoulderTargetAngle; }
    public double getExtenderTargetPos(){ return extenderTargetPos; }
    public  void setExtendTargetPos(double t){ extenderTargetPos = Math.min(3075/EXTEND_TICKS_PER_METER,Math.max(t, 0)); }
    public boolean nearTargetShoulder(){
        if ((Math.abs( getShoulderAngle()- getShoulderTargetAngle()))<2) return true;
        else return false;
    }

    public double getShoulderAngle(){ return shoulderAngle;}
    public double getExtendMeters(){return extendMeters;}
    public double getExtendInches(){return extendMeters * INCHES_PER_METER; }

    @Override
    public Map<String, Object> getTelemetry(boolean debug) {
        Map<String, Object> telemetryMap = new LinkedHashMap<>();

        telemetryMap.put("Current Articulation", articulation);
        telemetryMap.put("Pickup Stage", pickupConeStage);
        telemetryMap.put("Drop Stage", dropConeStage);

        if (debug) {
            telemetryMap.put("Robot X", robot.driveTrain.getPoseEstimate().getX());
            telemetryMap.put("Robot Y", robot.driveTrain.getPoseEstimate().getY());
            telemetryMap.put("Turret X", robot.turret.getTurretPosition(robot.driveTrain.getPoseEstimate()).getX());
            telemetryMap.put("Turret Y", robot.turret.getTurretPosition(robot.driveTrain.getPoseEstimate()).getY());
            telemetryMap.put("Axle X", axlePos.getX());
            telemetryMap.put("Axle Y", axlePos.getY());
            telemetryMap.put("Target Distance", targetDistance);
            telemetryMap.put("Target Height", targetHeight);
            telemetryMap.put("Target Turret Angle", targetTurretAngle);
            telemetryMap.put("Target Shoulder Angle", shoulderTargetAngle /SHOULDER_TICKS_PER_DEGREE);
            telemetryMap.put("Target Extension", extenderTargetPos/EXTEND_TICKS_PER_METER);
            telemetryMap.put("Target Angle", angle);
            telemetryMap.put("Length", length);
            telemetryMap.put("Distance", getDistance());
            telemetryMap.put("Height", getHeight());
            telemetryMap.put("Calibrate Stage", calibrateStage);
            telemetryMap.put("Bulb Pos", bulbGripped);
            telemetryMap.put("Extend Meters", extendMeters);
            telemetryMap.put("Extend Tics", extendPosition);
            telemetryMap.put("Extend Amps", extenderAmps);
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

        }else{

        }
        return telemetryMap;
    }

}
