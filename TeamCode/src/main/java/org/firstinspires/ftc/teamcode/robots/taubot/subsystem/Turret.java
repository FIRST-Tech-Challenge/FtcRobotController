package org.firstinspires.ftc.teamcode.robots.taubot.subsystem;

import static org.firstinspires.ftc.teamcode.robots.reachRefactor.util.Constants.USE_MOTOR_SMOOTHING;
import static org.firstinspires.ftc.teamcode.robots.taubot.util.Utils.wrapAngle;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.robots.reachRefactor.simulation.DcMotorExSim;
import org.firstinspires.ftc.teamcode.util.PIDController;

import java.util.LinkedHashMap;
import java.util.Map;

@Config(value = "PPTurret")
public class Turret implements Subsystem {

    public static PIDCoefficients TURRET_PID = new PIDCoefficients(0.02, 0.02, 0.002); //0.02, 0.01, 0.05)
    public static final double TICKS_PER_DEGREE = 36;
    public static double TURRET_TOLERANCE = 1;

    private final boolean simulated;

    private DcMotorEx motor;

    private PIDController turretPID;

    private double heading, targetHeading, power;

    BNO055IMU turretIMU;

    Orientation imuAngles;

    public Turret(HardwareMap hardwareMap, boolean simulated) {
        this.simulated = simulated;
        motor = simulated ? new DcMotorExSim(USE_MOTOR_SMOOTHING) : hardwareMap.get(DcMotorEx.class, "turret");
        turretIMU = hardwareMap.get(BNO055IMU.class, "turretIMU");
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        turretPID = new PIDController(TURRET_PID);
        turretPID.setInputRange(-360, 360);
        turretPID.setOutputRange(-1.0, 1.0);
        turretPID.setTolerance(TURRET_TOLERANCE);
        turretPID.enableIntegralZeroCrossingReset(false);
        turretPID.setIntegralCutIn(5); //suppress integral until within 5 degrees of target
        turretPID.enable();
        initIMU(turretIMU);
    }

    public void initIMU(BNO055IMU turretIMU){

        //setup Turret IMU
        BNO055IMU.Parameters parametersIMUTurret = new BNO055IMU.Parameters();
        parametersIMUTurret.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parametersIMUTurret.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parametersIMUTurret.loggingEnabled = true;
        parametersIMUTurret.loggingTag = "turretIMU";

        /*
        *  imu = hardwareMap.get(BNO055IMU.class, "baseIMU");
                BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
                parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
                imu.initialize(parameters);
                //because the Expansion hub is upsidedown:
                BNO055IMUUtil.remapZAxis(imu, AxisDirection.NEG_Y);*/

        turretIMU.initialize(parametersIMUTurret);
        this.turretIMU=turretIMU;

    }

    boolean initialized = false;
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

    public double getCorrection(){
        return  correction;
    }

    public void update(Canvas fieldOverlay) {

        imuAngles= turretIMU.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);

        if (!initialized) {
            //first time in - we assume that the robot has not started moving and that orientation values are set to the current absolute orientation
            //so first set of imu readings are effectively offsets
            offsetHeading = (heading-imuAngles.firstAngle)% 360;
            initialized = true;
        }
        //offset = heading - initialHeading
        //update current IMU heading before doing any other calculations
        heading = wrapAngle(offsetHeading + imuAngles.firstAngle);


        turretPID.setPID(TURRET_PID);
        turretPID.setTolerance(TURRET_TOLERANCE);
        turretPID.setSetpoint(0);
        turretPID.setInput(-distanceBetweenAngles(heading,targetHeading));
        correction = turretPID.performPID();
        //power = turretPID.onTarget() ? 0 : correction; //what was this? artificially stills micro corrections
        motor.setPower(correction);
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

    public double getHeading() {
        return heading;
    }

    /**
     * assign the current heading of the robot to a specific angle
     * @param angle the value that the current heading will be assigned to
     */
    public void setHeading(double angle){
        heading = angle;
        initialized = false; //triggers recalc of heading offset at next IMU update cycle
    }

    public static double localX = -5;

    public Pose2d getTurretPosition(Pose2d robot){

        double robotHeading = robot.getHeading();

        double robotX = robot.getX();
        double robotY = robot.getY();

        double worldX = robotX + (localX)*Math.cos(-robotHeading);
        double worldY = robotY - (localX)*Math.sin(-robotHeading);

       return new Pose2d(worldX,worldY,heading);
    }

    public boolean isTurretNearTarget(){
        return turretPID.onTarget();
    }

    @Override
    public Map<String, Object> getTelemetry(boolean debug) {
        Map<String, Object> telemetryMap = new LinkedHashMap<>();
        if(debug) {
            telemetryMap.put("turret heading", heading);
            telemetryMap.put("target turret heading", targetHeading);
            telemetryMap.put("turret motor amps", motor.getCurrent(CurrentUnit.AMPS));
            telemetryMap.put("turret near target", isTurretNearTarget());
            telemetryMap.put("turret correction", power);
        }

        return telemetryMap;
    }

    @Override
    public String getTelemetryName() {
        return "Turret";
    }
}


