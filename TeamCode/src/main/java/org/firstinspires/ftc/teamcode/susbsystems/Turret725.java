package org.firstinspires.ftc.teamcode.susbsystems;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.util.Math.Vector2;
import org.firstinspires.ftc.teamcode.util.Math.Vector3;
import org.firstinspires.ftc.teamcode.util.Util;

import java.util.ArrayList;
import java.util.List;

//TODO 7/14/25 - Frank
// - confirm default 0 degree position and rotation direction
//    - affects GoTo()
// - detect if a given point is too close to the robot
// - integrate with extension system for a getTargetVector() function
// - latestDirection vector is a clunky implementation. I should streamline it
//    - current solution: add a static method to Vector2 to create a new polar vector
// - move getVectorTarget() to the full system class, as it needs arm and extension information


public class Turret725 {

    //Turret Constants
    /*
    * Possible Turret Positions
    * .315 0 degrees
    * .279 45
    * .255 90
    * .241 135
    * .216 180
    * .195 225
    *
    * */
    //POTS Constants
    //old values with other sensor
    /*private static final double ZERO_DEGREES = .315;
    private static final double FORTYFIVE_DEGREES = .279;
    private static final double NINETY_DEGREES = .255;
    private static final double ONETHIRTYFIVE_DEGREES = .241;
    private static final double ONEEIGHTY_DEGREES = .216;
    private static final double TWOTWENTYFIVE_DEGREES = .195;
    // turret min/max  analog potentiometer values
    private static final double TURRET_MIN_POSITION = .185;
    private static final double TURRET_MAX_POSITION = .315;*/
    //pid for movement
    public static final double TURRET_PID_P_DEFAULT = 0.001;
    public static final double TURRET_PID_I_DEFAULT = 0;
    public static final double TURRET_PID_D_DEFAULT = 0.00003;
    private double pCoef = TURRET_PID_P_DEFAULT, iCoef = TURRET_PID_I_DEFAULT, dCoef = TURRET_PID_D_DEFAULT;
    //2nd pid values for movement if needed
    public static final double S_PID_P_DEFAULT = 0.0015;
    public static final double S_PID_I_DEFAULT = 0;
    public static final double S_PID_D_DEFAULT = 0.000045;


    private static final double TURRET_ERROR = .001;
    //changed this variable from TICKS_TO_DEG to DEG_TO_TICKS for accuracy
    //private final double TURRET_POT_TO_DEG = 0.000483271;
    private final double TURRET_LENGTH = 9;

    private DcMotorEx motor;

    private AnalogInput turretPot;
    private boolean isBusy = false;
    private boolean atHome, homed = false, isHoming = false;
    private double homePower = 0.2;
    private ElapsedTime homeTime = new ElapsedTime();
    private double targetPosition = Constants.TURRET_MAX_POSITION;
    private double currentPosition;
    private double currentPower = 0;
    private double currentAngle = 0;
    private double currentAngleRAD = 0;
    private double newcurrentAngleDegrees = 0;
    private double currentDegree;
    //represents the direction the turret has last been set to
    private ArrayList<Double> POTS_POSITIONS;
    private ArrayList<Double> POTS_ANGLES;

    private PIDController controller;

    public boolean AUTOSTOP = true;
    private Position currentxyzPosition;

    public Turret725(HardwareMap hm){
        init(hm);
        setPID(pCoef, iCoef, dCoef);
    }
    private void init(HardwareMap hm) {
        motor = hm.get(DcMotorEx.class, "turret");
        turretPot = hm.get(AnalogInput.class, "pot1");
        controller = new PIDController(pCoef, iCoef, dCoef);
        currentxyzPosition = new Position(DistanceUnit.INCH,0,0,0,System.nanoTime());
        POTS_POSITIONS = new ArrayList<Double>();
        POTS_POSITIONS.add(Constants.ZERO_DEGREES);
        POTS_POSITIONS.add(Constants.FORTYFIVE_DEGREES);
        POTS_POSITIONS.add(Constants.NINETY_DEGREES);
        POTS_POSITIONS.add(Constants.ONETHIRTYFIVE_DEGREES);
        POTS_POSITIONS.add(Constants.ONEEIGHTY_DEGREES);
        POTS_POSITIONS.add(Constants.TWOTWENTYFIVE_DEGREES);
        POTS_ANGLES = new ArrayList<Double>();
        POTS_ANGLES.add(0.0);
        POTS_ANGLES.add(45.0);
        POTS_ANGLES.add(90.0);
        POTS_ANGLES.add(135.0);
        POTS_ANGLES.add(180.0);
        POTS_ANGLES.add(225.0);
    }
    public void setPID(double P, double I, double D){
        pCoef = P; iCoef = I; dCoef = D;
    }
    
    public void StartHome() {
        homePower = -homePower;
        isBusy = true;
        isHoming = true;
        homeTime.reset();
    }

    public void GoTo(double targetPOTS) {
        targetPosition = targetPOTS;
        //safety lock out
        if (targetPosition <= Constants.TURRET_MIN_POSITION) {
            targetPosition = Constants.TURRET_MIN_POSITION;
        }
        if (targetPosition >= Constants.TURRET_MAX_POSITION) {
            targetPosition = Constants.TURRET_MAX_POSITION;
        }
        isBusy = true;
    }

    public double GetTargetPosition(){return targetPosition;}

    public double GetPos() {return (currentAngle);}
    public double GetRawPos() {return turretPot.getVoltage();}
    //public double GetTargetPos() {return GetRawPos();}
    public double GetRawTargetPos() {return targetPosition;}
    public boolean InError() { return Math.abs(currentPosition - GetRawTargetPos()) < TURRET_ERROR;}
    public boolean IsBusy() {return isBusy;}
    private void rawSet(double power) {
        motor.setPower(Util.IntClamp(power));
    }
    public void SetPower(double power) {
        isBusy = false;
        currentPower = power;
    }
    public void Stop() {
        rawSet(0);
        isBusy = false;
        isHoming = false;
        currentPower = 0;
    }
    private void setAngle(){
        double closestValue = POTS_POSITIONS.get(0);
        double minDifference = Math.abs(currentPosition-closestValue);
        for(double POTS : POTS_POSITIONS){
            double currentDifference = Math.abs(currentPosition - POTS);
            if (currentDifference < minDifference){
                minDifference = currentDifference;
                closestValue = POTS;
            }
        }
        currentAngle = POTS_ANGLES.get(POTS_POSITIONS.indexOf(closestValue));
        currentAngleRAD = Math.toRadians(currentAngle);
    }
    public void Update() {
        currentPosition = GetRawPos();
        newcurrentAngleDegrees = -53.268 * Math.pow(currentPosition,2) + 312.4707*currentPosition - 172.2134;
        setAngle();
        this.currentxyzPosition = new Position(DistanceUnit.INCH,TURRET_LENGTH * Math.cos(currentAngleRAD),TURRET_LENGTH * Math.sin(currentAngleRAD),0,System.nanoTime());

        homed = currentPosition <= Constants.TURRET_MIN_POSITION-TURRET_ERROR;
        if (isHoming && homed) {Stop();}

        if(isBusy && ((Math.abs(currentPosition - GetRawTargetPos()) < TURRET_ERROR))){Stop();};


        if (isBusy) {
            if (isHoming && !homed) {
                    currentPower = homePower;
                    //if (homeTime.seconds() > 3) StartHome();
            } else {
                if(targetPosition>currentPosition)
                    currentPower = .18;
                else{
                    currentPower = - .18;
                }
            }
        }
        //safety lockouts
        if (currentPosition >= Constants.TURRET_MAX_POSITION) {
            currentPower = Math.min(Math.max(currentPower, -1), 0);
        }
        if (currentPosition <= Constants.TURRET_MIN_POSITION) {
            currentPower = Math.min(Math.max(currentPower, 0), 1);
        }
        rawSet(currentPower);
    }
    public void SetPIDCoef(double p,double i,double d) {pCoef = p; iCoef = i; dCoef = d;}
    public double GetPower() {return motor.getPower();}
    public boolean Homed() {return homed;}
    public Position getPosition(){return currentxyzPosition;}
    public double getNewcurrentAngleDegrees() {return newcurrentAngleDegrees;}
}
