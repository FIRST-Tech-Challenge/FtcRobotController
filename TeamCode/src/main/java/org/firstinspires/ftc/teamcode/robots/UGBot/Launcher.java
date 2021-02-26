package org.firstinspires.ftc.teamcode.robots.UGBot;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.robots.UGBot.utils.Constants;
import org.firstinspires.ftc.teamcode.util.PIDController;

import static org.firstinspires.ftc.teamcode.util.Conversions.servoNormalize;

/**
 * Created by 2938061 on 11/10/2017.
 */
@Config
public class Launcher {

    //misc
    public double ticksPerDegree = 15.7;
    public boolean active = true;

    //actuators
    DcMotor elbow = null;
    DcMotorEx flywheelMotor = null;
    Servo servoTrigger = null;

    //flywheel variables
    public long lastUpdateTime;
    public int lastFlywheelPosition;
    PIDController flyWheelPID;
    double FlywheelCorrection = 0.00; //correction to apply to extention motor
    boolean FlywheelActivePID = true;
    double flywheelTargetTPS = 0;
    double FlywheelPwr = 1;
    double flywheelTPS;

    //elbow variables
    PIDController elbowPID;
    public static double kpElbow = 0.006; //proportional constant multiplier goodish
    public static  double kiElbow = 0.0; //integral constant multiplier
    public static  double kdElbow= 0.0; //derivative constant multiplier
    double elbowCorrection = 0.00; //correction to apply to elbow motor
    boolean elbowActivePID = true;
    int elbowPos = 0;
    double elbowPwr = 0;

    //elbow safety limits
    public int elbowMin = -50;
    public int elbowStart = 180; //put arm just under 18" from ground
    public int elbowLow = 300;
    public int elbowMinCalibration = -1340; //measure this by turning on the robot with the elbow fully opened and then physically push it down to the fully closed position and read the encoder value, dropping the minus sign
    public int actualElbowMax = 1465;
    public int elbowMid = (actualElbowMax + elbowMin)/2;
    public int elbowMaxSafetyOffset = 70; //makes sure that the robot doesn't try and extend to the elbow max exactly

    public Launcher(DcMotor elbow, DcMotorEx flywheelMotor, Servo servoTrigger){

        this.elbow = elbow;
        this.flywheelMotor = flywheelMotor;
        this.servoTrigger = servoTrigger;

        this.elbow.setTargetPosition(elbow.getCurrentPosition());
        this.elbow.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        this.flywheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        //PID
        lastUpdateTime = System.currentTimeMillis();
        lastFlywheelPosition = flywheelMotor.getCurrentPosition();
        flyWheelPID = new PIDController(0,0,0);
        elbowPID = new PIDController(0,0,0);
        elbowPID.setIntegralCutIn(40);
        elbowPID.enableIntegralZeroCrossingReset(false);

        servoTrigger.setPosition(servoNormalize(1800));

    }

    //Important junk

    long prevNanoTime;
    int prevMotorTicks;
    int prevOverride = Constants.overrideTPS;

    public void update(){
        if(active) {
            if(elbowActivePID)
                movePIDElbow(kpElbow, kiElbow, kdElbow, elbow.getCurrentPosition(), elbowPos);
            else
                elbowPos = elbow.getCurrentPosition();



            flywheelTPS = (flywheelMotor.getCurrentPosition() - prevMotorTicks) / ((System.nanoTime() - prevNanoTime) / 1E9);

            prevNanoTime = System.nanoTime();
            prevMotorTicks = flywheelMotor.getCurrentPosition();

            if (FlywheelActivePID) {
                spinPIDFlywheel(Constants.kpFlywheel, Constants.kiFlywheel, Constants.kdFlywheel, flywheelTPS, flywheelTargetTPS);
            }
            else{
                flywheelMotor.setPower(0);
            }

//

        }
    }

    public void stopAll(){
        setElbowPwr(0);
        setElbowActivePID(false);
        setFlywheelActivePID(false);
        update();
        active = false;
    }
    public void restart(double elbowPwr, double extendABobPwr){
        setElbowPwr(elbowPwr);
        active = true;
    }

    public void setActive(boolean active){this.active = active;}

    public void resetEncoders() {
        //just encoders - only safe to call if we know collector is in normal starting position
        elbow.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheelMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elbow.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        flywheelMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }


    //flywheel methods



    public void spinPIDFlywheel(double Kp, double Ki, double Kd, double currentVelocity, double targetVelocity) {

        //initialization of the PID calculator's output range, target value and multipliers
        flyWheelPID.setOutputRange(-FlywheelPwr, FlywheelPwr);
        flyWheelPID.setPID(Kp, Ki, Kd);
        flyWheelPID.setSetpoint(targetVelocity);
        flyWheelPID.enable();

        //initialization of the PID calculator's input range and current value
        //extendPID.setInputRange(0, 360);
        //extendPID.setContinuous();
        flyWheelPID.setInput(currentVelocity);

        //calculates the correction to apply
        FlywheelCorrection = flyWheelPID.performPID();

        //performs the extension with the correction applied
        flywheelMotor.setPower(FlywheelCorrection);
    }

    public double getFlywheelTPS() {
        return flywheelTPS;
    }


    //elbow methods



    public void movePIDElbow(double Kp, double Ki, double Kd, double currentTicks, double targetTicks) {

        //initialization of the PID calculator's output range, target value and multipliers
        elbowPID.setOutputRange(-elbowPwr, elbowPwr);
        elbowPID.setPID(Kp, Ki, Kd);
        elbowPID.setSetpoint(targetTicks);
        elbowPID.enable();

        //initialization of the PID calculator's input range and current value
        elbowPID.setInput(currentTicks);

        //calculates the correction to apply
        elbowCorrection = elbowPID.performPID();

        //moves elbow with the correction applied
        elbow.setPower(elbowCorrection);
    }

    public void setElbowActivePID(boolean isActive){elbowActivePID = isActive;}

    public void setFlywheelActivePID(boolean isActive){FlywheelActivePID = isActive;}

    private void setElbowTargetPos(int pos){elbowPos = Math.min(Math.max(pos, elbowMin), actualElbowMax -elbowMaxSafetyOffset);}

    public void setElbowTargetPosNoCap(int pos){elbowPos = pos;}

    public boolean setElbowTargetPos(int pos, double speed){
        setElbowTargetPos(pos);
        setElbowPwr(speed);
        if (nearTargetElbow()) return true;
        else return false;
    }


    public boolean setElbowTargetAngle(double angleDegrees){
        setElbowTargetPos((int) (ticksPerDegree* angleDegrees) + 141); //plus the offset to zero
        return true;
    }

    public int getElbowTargetPos(){
        return elbowPos;
    }
    public int getElbowCurrentPos() {
        return elbow.getCurrentPosition();
    }

    public double getFlywheelTargetTPS() {return flywheelTargetTPS;}
    public void setFlywheelTargetTPS(double flywheelTargetTPS) {this.flywheelTargetTPS = flywheelTargetTPS;}

    public double getCurrentAngle(){return  elbow.getCurrentPosition()/ticksPerDegree;}

    public void setElbowPwr(double pwr){ elbowPwr = pwr; }

    public boolean nearTargetElbow(){
        if ((Math.abs( getElbowCurrentPos()-getElbowTargetPos()))<55) return true;
        else return false;
    }

    public void adjustElbowAngle(double speed){
        setElbowTargetPos(getElbowCurrentPos() + (int)(150 * speed));
    }

    public void decreaseElbowAngle(){
        setElbowTargetPos(Math.max(getElbowCurrentPos() - 100, 0));
    }

}

