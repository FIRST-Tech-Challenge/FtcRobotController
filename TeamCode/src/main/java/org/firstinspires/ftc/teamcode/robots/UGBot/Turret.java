
//written by Cooper Clem, 2019

package org.firstinspires.ftc.teamcode.robots.UGBot;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.robots.UGBot.utils.Constants;
import org.firstinspires.ftc.teamcode.util.Conversions;
import org.firstinspires.ftc.teamcode.util.PIDController;
import org.opencv.core.Mat;

import static org.firstinspires.ftc.teamcode.util.Conversions.diffAngle;
import static org.firstinspires.ftc.teamcode.util.Conversions.diffAngle2;
import static org.firstinspires.ftc.teamcode.util.Conversions.nextCardinal;
import static org.firstinspires.ftc.teamcode.util.Conversions.wrap360;
import static org.firstinspires.ftc.teamcode.util.Conversions.wrapAngle;
import static org.firstinspires.ftc.teamcode.util.Conversions.wrapAngleMinus;

@Config
public class Turret{
    //motor
    private  DcMotor motor = null;
    private double motorPwr = 1;
    long turnTimer;
    boolean turnTimerInit;
    private double minTurnError = 1.0;
    private boolean active = true;

    //PID
    PIDController turretPID;
    public static double kpTurret = 0.03; //proportional constant multiplier goodish
    public static  double kiTurret = .01; //integral constant multiplier
    public static  double kdTurret= .05; //derivative constant multiplier
    double correction = 0.00; //correction to apply to turret motor

    //IMU
    BNO055IMU turretIMU;
    double turretRoll;
    double turretPitch;
    double turretHeading;
    boolean initialized = false;
    private double offsetHeading;
    private double offsetRoll;
    private double offsetPitch;
    private double turretTargetHeading = 0.0;
    Orientation imuAngles;
    boolean maintainHeadingInit;
    private final double angleIncrement = 10;

    //sensors
    //DigitalChannel magSensor;

    //a single supported Danger Zone - where the chassis is not allow to point when it is active
    private boolean dangerModeActive = false;
    private double dangerZoneCenter = Constants.DANGER_ZONE_CENTER;
    private double dangerZoneWidth = Constants.DANGER_ZONE_WIDTH;



    public Turret(DcMotor motor, BNO055IMU turretIMU) {

        //motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //motor.setTargetPosition(motor.getCurrentPosition());
        //motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //this.magSensor = magSensor;

        this.motor = motor;
        turretTargetHeading = 0.0;
        turretPID = new PIDController(0,0,0);
        initIMU(turretIMU);
    }

    public void initIMU(BNO055IMU turretIMU){

        //setup Turret IMU
        BNO055IMU.Parameters parametersIMUTurret = new BNO055IMU.Parameters();
        parametersIMUTurret.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parametersIMUTurret.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parametersIMUTurret.loggingEnabled = true;
        parametersIMUTurret.loggingTag = "turretIMU";


        turretIMU.initialize(parametersIMUTurret);
        this.turretIMU=turretIMU;

    }

    double baseHeading = 0.0;
    public void update(double baseHeading){


        //IMU Update
        imuAngles= turretIMU.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
        if (!initialized) {
            //first time in - we assume that the robot has not started moving and that orientation values are set to the current absolute orientation
            //so first set of imu readings are effectively offsets
            offsetHeading = wrapAngleMinus(360 - imuAngles.firstAngle, turretHeading);
            offsetRoll = wrapAngleMinus(imuAngles.secondAngle, turretRoll);
            offsetPitch = wrapAngleMinus(imuAngles.thirdAngle, turretPitch);
            initialized = true;
        }

        //update current heading before doing any other calculations
        turretHeading = wrapAngle((360-imuAngles.firstAngle), offsetHeading);

        remapHeadingToSafe(turretTargetHeading);

        this.baseHeading = baseHeading;

//        double degreesOfSeparationForBase = diffAngle2(turretHeading, baseHeading);
//        double degreesOfSeparationForTarget = diffAngle2(turretHeading, turretTargetHeading);
//        double dangerLeft = wrapAngleMinus(baseHeading,Constants.DANGER_ZONE_WIDTH/2);
//        double dangerRight = (baseHeading + Constants.DANGER_ZONE_WIDTH/2) % 360;
//        double theMrVsSpecialVariable = Math.min(diffAngle2(turretHeading,dangerLeft), diffAngle2(turretHeading,dangerRight));
//        double directionOfTurn = degreesOfSeparationForTarget - 180;
//        double directionOfDanger = degreesOfSeparationForBase - 180;
//
//        if(between360(turretTargetHeading, dangerLeft, dangerRight)){
//            if(directionOfDanger > 0){
//                turretTargetHeading = (dangerLeft - Constants.DANGER_ZONE_WIDTH/4) % 360;
//            }
//            else{
//                turretTargetHeading = (dangerRight + Constants.DANGER_ZONE_WIDTH/4) % 360;
//            }
//        }
//
//        if(between360(dangerLeft, turretHeading, turretTargetHeading) && between360(dangerRight, turretHeading, turretTargetHeading)){
//            if(directionOfTurn > 0){
//                turretTargetHeading = (turretHeading + 20) % 360;
//            }
//            else{
//                turretTargetHeading = (turretHeading - 20) % 360;
//            }
//        }

        if(active) {
            maintainHeadingTurret();
        }
        else
            motor.setPower(0);
    }

    //returns whether the turretTargetHeading is within the passed in danger zone
    public boolean isInDangerZone(){
        return Conversions.between360(turretTargetHeading,getDangerZoneLeft(), getDangerZoneRight());
    }

    public double remapHeadingToSafe(double heading){
        if(isInDangerZone() && dangerModeActive){
            if(-diffAngle2(getDangerZoneCenter(),heading) > 0){
                return wrap360(getDangerZoneRight(),Constants.DANGER_ZONE_SAFTEY_BUFFER);
            }
            else{
                return wrap360(getDangerZoneLeft(),-Constants.DANGER_ZONE_SAFTEY_BUFFER);
            }
        }
        else{
            return heading;
        }
    }

    public boolean crossesDangerZone(){
        double targetDist = Math.abs(diffAngle2(turretHeading,turretTargetHeading));

        return (Math.abs(diffAngle2(turretHeading, getDangerZoneLeft())) <  targetDist
                && Math.abs(diffAngle2(turretHeading, getDangerZoneRight())) <  targetDist
                && targetDist < 180);
    }

    private int directionToDZ(){
        if(-diffAngle2(turretHeading, dangerZoneCenter) < 0){
            return -1;
        }
        else{
            return 1;
        }
    }

    public double approachSafe(double heading){
        if(crossesDangerZone() && dangerModeActive){
            return directionToDZ() * 170;
        }
        else{
            return 0;
        }
    }

    public boolean isDangerModeActive(){return dangerModeActive;}

    public void setDangerModeActive(boolean dangerModeActive){this.dangerModeActive = dangerModeActive;}

    public double getDangerZoneCenter(){return dangerZoneCenter;}

    public void setDangerZoneCenter(double dangerZoneCenter){this.dangerZoneCenter = dangerZoneCenter;}

    public double getDangerZoneWidth(){return dangerZoneWidth;}

    public void setDangerZoneWidth(double dangerZoneWidth){this.dangerZoneWidth = dangerZoneWidth;}

    public double getDangerZoneLeft(){return wrap360(dangerZoneCenter, -dangerZoneWidth / 2);}

    public double getDangerZoneRight(){return wrap360(dangerZoneCenter, dangerZoneWidth / 2);}


    /**
     * assign the current heading of the robot to a specific angle
     * @param angle the value that the current heading will be assigned to
     */
    public void setHeading(double angle){
        turretHeading = angle;
        initialized = false; //triggers recalc of heading offset at next IMU update cycle
    }

    //public boolean getMagSensorVal() {return magSensor.getState(); }

    public boolean isActive(){
        return active;
    }

    public void setActive(boolean active){this.active = active;}

    public void adjust(double speed) {
        setTurntableAngle(getHeading(), 7.0 * speed);
    }

    public void rotateRight(double speed) {
        setTurntableAngle(getHeading(), angleIncrement * speed);
    }

    public void rotateLeft(double speed){

        setTurntableAngle(getHeading(), angleIncrement * -speed);

    }


    public boolean rotateCardinalTurret(boolean right){

        setTurntableAngle(nextCardinal(getHeading(),right,10));

        return true;
    }

    public void stopAll() {
        setPower(0);
        active = false;
    }

    public void setTurntableAngle(double currentAngle, double adjustAngle){
        turretTargetHeading=wrap360(currentAngle, adjustAngle);
    }

    public boolean setTurntableAngle(double angle){
        turretTargetHeading=wrap360(angle);
        return Conversions.between(getHeading(), angle - Constants.TURRET_TOLERANCE, angle + Constants.TURRET_TOLERANCE);
    }

    public void setPower(double pwr){
        motorPwr = pwr;
        motor.setPower(pwr);
    }

    double turnError = 0;
    public void movePIDTurret(double Kp, double Ki, double Kd, double currentAngle, double targetAngle) {

        //initialization of the PID calculator's output range, target value and multipliers
        turretPID.setOutputRange(-1, 1);
        turretPID.setPID(Kp, Ki, Kd);
        turretPID.setSetpoint(targetAngle);
        turretPID.enable();

        //initialization of the PID calculator's input range and current value
        turretPID.setInputRange(0, 360);
        turretPID.setContinuous();
        turretPID.setInput(currentAngle);

        turnError = diffAngle2(targetAngle, currentAngle);

        //calculates the angular correction to apply
        correction = turretPID.performPID();

        //performs the turn with the correction applied
        setPower(correction);
    }



    public void setTurretMotorMode(boolean IMUMODE){
        if(IMUMODE) {motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);}
        else{motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);}

    }
    public void maintainHeadingTurret() {
        //if this is the first time the button has been down, then save the heading that the robot will hold at and set a variable to tell that the heading has been saved
        if (!maintainHeadingInit) {
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            //turretTargetHeading = turretHeading;
            maintainHeadingInit = true;
        }

        if (currentMode == TurretMode.chassisRelative) {
            turretTargetHeading = toChassisRelative();
        }

        movePIDTurret(kpTurret, kiTurret, kdTurret, turretHeading, turretTargetHeading + approachSafe(baseHeading));
    }


    public double getHeading(){
        return turretHeading;
    }
    public double getTurretTargetHeading(){
        return turretTargetHeading;
    }
    public double getCorrection(){return correction;}
    public double getMotorPwr(){return motorPwr;}
    public double getMotorPwrActual(){return motor.getPower();}

    public TurretMode getCurrentMode() {
        return currentMode;
    }
    private TurretMode previousMode = TurretMode.fieldRelative; //default mode
    public void setCurrentMode(TurretMode mode) {
        this.currentMode = mode;
        if (previousMode!=currentMode){ //we've just changed modes
            if (currentMode == TurretMode.fieldRelative) {
                //on entering fieldRelative from chassisRelative, stop movement and set the current target value
                turretTargetHeading = toFieldRelative();
            }
            if (currentMode == TurretMode.fieldRelative) {
                //on entering chassisRelative from fieldRelative, stop movement and set the current target value
                turretTargetHeading = toChassisRelative();
            }
            previousMode=currentMode;

        }
    }

    private double toFieldRelative() {return turretHeading;}

    private double toChassisRelative() {return wrap360(baseHeading,turretHeading);}

    private TurretMode currentMode = TurretMode.fieldRelative;
    public enum TurretMode {
        fieldRelative, // normal mode - requested angles are relative to the field (starting orientation of turret imu)
        chassisRelative // requested angles are relative to the chassis heading
    }



}
