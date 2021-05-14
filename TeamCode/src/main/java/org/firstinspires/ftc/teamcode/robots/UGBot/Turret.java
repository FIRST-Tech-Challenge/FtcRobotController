
//written by Cooper Clem, 2019

package org.firstinspires.ftc.teamcode.robots.UGBot;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.robots.UGBot.utils.Constants;
import org.firstinspires.ftc.teamcode.util.PIDController;

import static org.firstinspires.ftc.teamcode.util.Conversions.between360Clockwise;
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
    private double turretChassisTarget = 0; //This is the target to set when you want to be in chassis relative mode
    Orientation imuAngles;
    boolean maintainHeadingInit;


    //sensors
    //DigitalChannel magSensor;

    //a single supported Danger Zone - where the chassis is not allowed to point when it is active
    private boolean dangerModeActive = false;
    private double dangerZoneCenter = Constants.DANGER_ZONE_CENTER;
    private double dangerZoneWidth = Constants.DANGER_ZONE_WIDTH;


    //manual steering
    private double manualOffset; //positive/negative angle by which drivers want the turret offset - should reset to zero each time the mode changes
    private final double speedBoost = 10/7; //boosted speed at which drivers change target heading through calls to rotateRight or rotateLeft


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

        this.baseHeading = baseHeading;

        if(active) {
            maintainHeadingTurret();
        }
        else
            motor.setPower(0);
    }

    //returns whether the turretTargetHeading is within the danger zone
    //this is currently calculated relative to the chassis center line
    public boolean isInDangerZone(){
        return between360Clockwise(turretTargetHeading,getDangerZoneLeft(), getDangerZoneRight());
    }

    public double remapHeadingToSafe(double heading){
        if(isInDangerZone() && isDangerModeActive()){
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

    //todo check this
    public boolean crossesDangerZone(){
        double targetDist = Math.abs(diffAngle2(turretHeading,turretTargetHeading));

        return (Math.abs(diffAngle2(turretHeading, getDangerZoneLeft())) <  targetDist
                && Math.abs(diffAngle2(turretHeading, getDangerZoneRight())) <  targetDist
                && targetDist < 180);
    }

    public int directionToDZ(){
        if(-diffAngle2(turretHeading, getDangerZoneCenter()) < 0){
            return -1;
        }
        else{
            return 1;
        }
    }


    public double approachSafe(double heading){
        if(crossesDangerZone() && isDangerModeActive()){
            //start taking the longer path so turret won't cross the danger zone
            return  wrap360((-directionToDZ() * 170),turretHeading);
        }
        else{
            return 0;
        }
    }

    public boolean isDangerModeActive(){return dangerModeActive;}

    public void setDangerModeActive(boolean dangerModeActive){this.dangerModeActive = dangerModeActive;}

    public double getDangerZoneCenter(){return wrap360(dangerZoneCenter, baseHeading);}

    public void setDangerZoneCenter(double dangerZoneCenter){this.dangerZoneCenter = dangerZoneCenter;}

    public double getDangerZoneWidth(){return dangerZoneWidth;}

    public void setDangerZoneWidth(double dangerZoneWidth){this.dangerZoneWidth = dangerZoneWidth;}

    public double getDangerZoneLeft(){return wrap360(getDangerZoneCenter(), -getDangerZoneWidth() / 2);}

    public double getDangerZoneRight(){return wrap360(getDangerZoneCenter(), getDangerZoneWidth() / 2);}

    public double getManualOffset() {
        return manualOffset;
    }

    public void setManualOffset(double manualOffset) {
        this.manualOffset = manualOffset;
    }



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

    //seems to have been used for finer/slower turret manual controls
//    public void adjust(double speed) {
//        setTurretOffset(getHeading(), 7.0 * speed);
//    }
    //speed here should be in degrees per second
    long lastTime = System.nanoTime();
    public void adjust(double speed) {

        // if last time this was called was more than x seconds ago reset - this must be a new application
        if (lastTime>2*1E9+System.nanoTime()) lastTime=System.nanoTime();
        long elapsedTime = System.nanoTime()-lastTime; //kind of assuming that the elapsed time represents roughly the current loop speed
        double timeFactoredSpeed = speed * elapsedTime/1E9; //this is actually angle difference to add, factored for time
        setManualOffset(getHeading() + manualOffset + timeFactoredSpeed);
    }

    public void rotateRight(double speed) {

        //setTurretOffset(getHeading()+ manualOffset, angleIncrement * speed);
        adjust(speedBoost *speed);
    }

    public void rotateLeft(double speed){

        //setTurretOffset(getHeading()+ manualOffset, angleIncrement * -speed);
        adjust(speedBoost *-speed);

    }

    //*************** WARNING ************************
    //rotate cardinal has not been updated to work with dangerZones or driverOffsets
    //only use if there is no chance of danger and in fieldRelativeMode
    public boolean rotateCardinalTurret(boolean right){

        setTurretAngle(nextCardinal(getHeading(),right,10));

        return true;
    }

    public void stopAll() {
        setPower(0);
        active = false;
    }

//    public void setTurretOffset(double currentAngle, double adjustAngle){
//         //setTurretAngle( wrap360(currentAngle, adjustAngle));
//    }

    public boolean setTurretAngle(double angle){
        if (currentMode==TurretMode.chassisRelative)
            turretChassisTarget = wrap360(angle);
        else
            turretTargetHeading=wrap360(angle);
        return isTurretNearTarget();
    }

    public boolean setTurretAngleNoCheck(double angle){
        turretTargetHeading=wrap360(angle);
        return true;
    }

    public boolean isTurretNearTarget(){
        return between360Clockwise(getHeading(), getTargetHeading() - Constants.TURRET_TOLERANCE, getTargetHeading() + Constants.TURRET_TOLERANCE);
    }


    private void setPower(double pwr){
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

    //todo: should we keep this old code? - we are now doing our own custom PID
    // and it's always IMU relative in some way, so this should not be used
    public void setTurretMotorMode(boolean IMUMODE){
        if(IMUMODE) {motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);}
        else{motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);}

    }

    public void maintainHeadingTurret() {

        //if this is the first time the button has been down, then save the heading that the robot will hold at
        // and set a variable to tell that the heading has been saved
        if (!maintainHeadingInit) {
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            //turretTargetHeading = turretHeading;
            maintainHeadingInit = true;
        }

        if (currentMode == TurretMode.chassisRelative) {
            //every time we come in, the targetHeading might need to shift if the base has rotated, regardless of changes to targetHeading
            turretTargetHeading = fromChassisRelative(turretChassisTarget);
        }


        turretTargetHeading = remapHeadingToSafe(turretTargetHeading);
        double offsetTargetHeading = wrap360(turretTargetHeading, getManualOffset()); //alter by driver input if any present
        offsetTargetHeading = remapHeadingToSafe(offsetTargetHeading); //remap safe since the offset could have put it back in the danger zone

        movePIDTurret(kpTurret, kiTurret, kdTurret, turretHeading, wrap360(offsetTargetHeading,approachSafe(baseHeading)));
    }


    public double getHeading(){
        if (currentMode==TurretMode.chassisRelative)
            return baseHeading;
        else
        return turretHeading;
    }

    public double getTargetHeading(){
        if (currentMode==TurretMode.chassisRelative)
            return turretChassisTarget;
        else
            return turretTargetHeading;
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
                turretTargetHeading = chassisToFieldRelative();
            }
            if (currentMode == TurretMode.chassisRelative) {
                //on entering chassisRelative from fieldRelative, stop movement and set the current target va
                turretTargetHeading = fieldToChassisRelative();
            }
            setManualOffset(0); //reset any driver offset any time TurretMode is changed
            previousMode=currentMode;

        }
    }

    private double chassisToFieldRelative() {return wrap360(turretHeading, baseHeading);}

    private double fieldToChassisRelative() {return wrap360(turretHeading, -baseHeading);}

    //convert the requested angle that is relative to chassis center/forward line
    //into a field-relative angle
    private double fromChassisRelative(double requestAngle){
        return wrap360(baseHeading, requestAngle);
    }

    private TurretMode currentMode = TurretMode.fieldRelative;
    public enum TurretMode {
        fieldRelative, // normal mode - requested angles are relative to the field (starting orientation of turret imu)
        chassisRelative, // requested angles are relative to the chassis headi
    }



}
