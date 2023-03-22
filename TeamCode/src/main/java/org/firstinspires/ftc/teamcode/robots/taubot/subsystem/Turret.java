package org.firstinspires.ftc.teamcode.robots.taubot.subsystem;

import static org.firstinspires.ftc.teamcode.robots.reachRefactor.util.Constants.USE_MOTOR_SMOOTHING;
import static org.firstinspires.ftc.teamcode.robots.taubot.util.Utils.wrapAngle;
import static org.firstinspires.ftc.teamcode.util.utilMethods.wrapAngleMinus;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.robots.UGBot.utils.Constants;
import org.firstinspires.ftc.teamcode.robots.reachRefactor.simulation.DcMotorExSim;
import org.firstinspires.ftc.teamcode.util.PIDController;

import java.util.LinkedHashMap;
import java.util.Map;

@Config(value = "PPTurret")
public class Turret implements Subsystem {

    public static PIDCoefficients TURRET_PID = new PIDCoefficients(0.01, 0.02, 0.002); //0.02, 0.01, 0.05)
    public static final double TICKS_PER_DEGREE = 36;
    public static double TURRET_TOLERANCE = 1;

    private final boolean simulated;

    private DcMotorEx motor;

    private PIDController turretPID;

    private double heading, targetHeading, power;

    private int targetTics; //when not in IMU mode, this is the current target
    private int TRANSFER_TICS = 1786;

    BNO055IMU turretIMU;

    Orientation imuAngles;
    boolean turretInitialized = false;

    private static double cacheHeading;

    private Robot robot;

    DigitalChannel turretIndex;

    public Turret( HardwareMap hardwareMap, Robot robot, boolean simulated) {
        this.robot = robot;
        this.simulated = simulated;
        motor = simulated ? new DcMotorExSim(USE_MOTOR_SMOOTHING) : hardwareMap.get(DcMotorEx.class, "turret");
        turretIMU = hardwareMap.get(BNO055IMU.class, "turretIMU");
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        turretIndex = hardwareMap.get(DigitalChannel.class, "turretIndex");
        turretIndex.setMode(DigitalChannel.Mode.INPUT);

        turretPID = new PIDController(TURRET_PID);
        turretPID.setInputRange(-360, 360);
        turretPID.setOutputRange(-1.0, 1.0);
        turretPID.setTolerance(TURRET_TOLERANCE);
        turretPID.enableIntegralZeroCrossingReset(true);
        turretPID.setIntegralCutIn(4); //suppress integral until within x degrees of target
        turretPID.enable();

        BNO055IMU.Parameters parametersIMUTurret = new BNO055IMU.Parameters();
        parametersIMUTurret.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parametersIMUTurret.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parametersIMUTurret.loggingEnabled = true;
        parametersIMUTurret.loggingTag = "turretIMU";
        turretIMU.initialize(parametersIMUTurret);

        articulation = Articulation.runToAngle;
    }

    double offsetHeading;

    void customWrapHeading(){
        if(heading > 180){
            heading -= 360;
            targetHeading -= 360;
        }
        if(heading < -180){
            heading += 360;
            targetHeading += 360;
        }
    }
    public static double distanceBetweenAngles(double currentAngle, double targetAngle){
        double result = wrapAngle(targetAngle - currentAngle);
        if(result >180) return result - 360;
        return result;
    }

    double correction = 0;
    double error = 0;

    public double getCorrection(){
        return  correction;
    }

    public void cacheHeadingForNextRun(){
        cacheHeading = heading;
    }

    Articulation articulation;

    public enum Articulation{
        runToAngle,
        lockToOneHundredAndEighty,
        transfer,
        lockToZero,
        calibrate
    }

    public Articulation articulate(Articulation target){
        articulation = target;

        switch (articulation){
            case calibrate:

                break;
            case runToAngle: //normal run to a target angle mode
                setControlMethodIMU(true);
                turretPID.setInput(-distanceBetweenAngles(heading,targetHeading));
                break;
            case lockToOneHundredAndEighty: //home position is facing facing back of robot not towards underarm
                targetTics = TRANSFER_TICS;
                setControlMethodIMU(false);
                //turretPID.setInput(-distanceBetweenAngles(heading,180 + Math.toDegrees(robot.driveTrain.getRawHeading())));
                break;
            case lockToZero: //fold is facing towards underarm
                targetTics = 0;
                setControlMethodIMU(false);
                turretPID.setInput(-distanceBetweenAngles(heading,Math.toDegrees(robot.driveTrain.getRawHeading())));
                break;
            case transfer: //transfer position is facing facing back of robot not towards underarm
                targetTics = TRANSFER_TICS;
                setControlMethodIMU(false);
                //turretPID.setInput(-distanceBetweenAngles(heading,180 + Math.toDegrees(robot.driveTrain.getRawHeading())));
                break;
        }

        return articulation;
    }

    public double getError(){
        return -distanceBetweenAngles(heading,targetHeading);
    }
    public void update(Canvas fieldOverlay) {

        articulate(articulation);

        imuAngles= turretIMU.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
        if (!turretInitialized) {
            //first time in - we assume that the robot has not started moving and that orientation values are set to the current absolute orientation
            //so first set of imu readings are effectively offsets
            //if (!nearZero(cacheHeading))
                //we have a cached heading from a prior run, so use that to create the current offset
                //todo - this has not been tested yet - also might want to do this differently - like cached heading should be stored and retrieved from disk
                //offsetHeading = wrapAngleMinus(imuAngles.firstAngle, cacheHeading);
            //else
                offsetHeading = wrapAngleMinus(imuAngles.firstAngle + Constants.TURRET_OFFSET_HEADING, heading);
            turretInitialized = true;
        }

        //update current IMU heading before doing any other calculations
        heading = wrapAngle(offsetHeading + imuAngles.firstAngle) ;
        cacheHeadingForNextRun();

        if (controlMethodIMU) {
            turretPID.setPID(TURRET_PID);
            turretPID.setTolerance(TURRET_TOLERANCE);
            turretPID.setSetpoint(0);
            correction = turretPID.performPID();
            error = turretPID.getError();
            //power = turretPID.onTarget() ? 0 : correction; //what was this? artificially stills micro corrections
        }
        else {

            motor.setTargetPosition(targetTics);
        }

        //not sure if this is still workable given the changes to using RunToPosition for
        if(Crane.robotIsNotTipping && turretPID.isEnabled() && controlMethodIMU) {
            motor.setPower(correction);
        }
    }

    public void stop() {
        setTargetHeading(heading);
    }

    public void setTargetHeading(double targetHeading){
        this.targetHeading = wrapAngle(targetHeading);
    }
    public boolean turnToTargetHeading(double targetHeading){
        setTargetHeading(targetHeading);
        return turretPID.getError() < 5;
    }

    public double getTargetHeading() {
        return targetHeading;
    }

    int calibrateStage = 0;
    long calibrateTimer = 0;

    public static double calibratePower = 250;

    public boolean calibrate(){
        switch (calibrateStage){
            case 0:
                turretPID.disable();
                articulate(Articulation.calibrate);
                motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                calibrateStage++;
                break;
            case 1:
                motor.setVelocity(calibratePower);
                if(!turretIndex.getState()){ //is magnet detectect?
                    //zeroHeading(LIMIT_SWITCH_ANGLE_OFFSET);
                    setHeading(LIMIT_SWITCH_ANGLE_OFFSET);
                    motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    turretPID.enable();
                    calibrateStage++;
                }
                break;
            case 2:
                articulate(Articulation.lockToZero);
                calibrateStage = 0;
                return true;
        }
        return false;
    }


    private boolean controlMethodIMU = true;

    public void setControlMethodIMU(boolean imu){
        if (imu) {
            //we are going to set IMU mode
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            controlMethodIMU = true;
        }
        else {

            //we are going to set run-to-position mode
            motor.setTargetPosition(motor.getCurrentPosition()); //encoder should have be reset at the end of calibration - here we set it to it's current position so it doesn't jerk
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor.setPower(1); //full power available
            motor.setVelocity(750); //ticks per second max velocity todo tune for moderate speed
            controlMethodIMU = false;
        }

    }

    //transfer in the turret subsystem means to rotate the turret to the transfer position and leave it there
    //we will turn off heading-based PID and instead use RunToPosition
    public boolean transfer(){
        return false;
    }
    boolean transferInit = false;
    int transferStage = 0;

    public void transferExit(){
        transferStage = 0;
        transferInit = false;
    }

    public static double LIMIT_SWITCH_ANGLE_OFFSET = 0;

    public double getHeading() {
        return heading;
    }

    /**
     * assign the current heading of the robot to a specific angle
     * @param angle the value that the current heading will be assigned to
     */
    public void setHeading(double angle){
        heading = angle;
        turretInitialized = false; //triggers recalc of heading offset at next IMU update cycle
    }

    public static double localX = -5;

    public static double robotRadius = 17.5/2;
    public static double turretRadius = 6;
    public static double turretOffset = robotRadius-turretRadius;
    public static double axleDistanceFromRotation = 5;


    public Pose2d getTurretPosition(){
        Pose2d robotPosition = robot.driveTrain.getPoseEstimate();
        return new Pose2d (robotPosition.getX()-turretOffset*Math.sin(robotPosition.getHeading() + Math.PI/2),robotPosition.getY()+turretOffset*Math.cos(robotPosition.getHeading()+ Math.PI/2));
    }

    public Pose2d getAxlePosition(){
        Pose2d turretPosition = getTurretPosition();
        return new Pose2d(turretPosition.getX() - axleDistanceFromRotation*Math.sin(Math.toRadians(heading)+ Math.PI/2), turretPosition.getY() + axleDistanceFromRotation*Math.cos(Math.toRadians(heading)+ Math.PI/2) );
    }

    public boolean isTurretNearTarget(){
        return turretPID.onTarget();
    }

    @Override
    public Map<String, Object> getTelemetry(boolean debug) {
        Map<String, Object> telemetryMap = new LinkedHashMap<>();
        telemetryMap.put("articulation", articulation);
        telemetryMap.put("turret heading", heading);
        telemetryMap.put("turret error", turretPID.getError());
        telemetryMap.put("turret tics", motor.getCurrentPosition());
        telemetryMap.put("turret power", motor.getPower());

        if(debug) {

            telemetryMap.put("target turret heading", targetHeading);
            telemetryMap.put("turret motor amps", motor.getCurrent(CurrentUnit.AMPS));
            telemetryMap.put("turret near target", isTurretNearTarget());
            telemetryMap.put("turret correction", power);
            telemetryMap.put("turret thing", turretIndex.getState());
            telemetryMap.put("turret calibrate", calibrateStage);
        }

        return telemetryMap;
    }

    @Override
    public String getTelemetryName() {
        return "Turret";
    }
}


