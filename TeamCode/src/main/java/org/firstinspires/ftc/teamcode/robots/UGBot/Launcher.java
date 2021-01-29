package org.firstinspires.ftc.teamcode.robots.UGBot;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.util.PIDController;

import static org.firstinspires.ftc.teamcode.util.Conversions.servoNormalize;

/**
 * Created by 2938061 on 11/10/2017.
 */
@Config
public class Launcher {

    //misc
    public double ticksPerDegree = 19.4705882353;//todo-remeasure
    public boolean active = true;

    //actuators
    DcMotor elbow = null;
    DcMotor flywheelMotor = null;
    Servo servoGripper = null;

    //flywheel variables
    PIDController flyWheelPID;
    public static double kpFlywheel = 0.006; //proportional constant multiplier goodish
    public static  double kiFlywheel = 0.0; //integral constant multiplier
    public static  double kdFlywheel= 0.0; //derivative constant multiplier
    double FlywheelCorrection = 0.00; //correction to apply to extention motor
    boolean FlywheelActivePID = true;
    int FlywheelRPM = 0;
    double FlywheelPwr = 0;

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

    public Launcher(DcMotor elbow, DcMotor flywheelMotor, Servo servoGripper){

        this.elbow = elbow;
        this.flywheelMotor = flywheelMotor;
        this.servoGripper = servoGripper;

        this.elbow.setTargetPosition(elbow.getCurrentPosition());
        this.elbow.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        this.flywheelMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        //PID
        flyWheelPID = new PIDController(0,0,0);
        elbowPID = new PIDController(0,0,0);
        elbowPID.setIntegralCutIn(40);
        elbowPID.enableIntegralZeroCrossingReset(false);


    }

    //Important junk

    public void update(){
        if(active) {
            if(elbowActivePID)
                movePIDElbow(kpElbow, kiElbow, kdElbow, elbow.getCurrentPosition(), elbowPos);
            else
                elbowPos = elbow.getCurrentPosition();

        }
        if(active) {
            if(FlywheelActivePID)
                movePIDExtend(kpFlywheel, kiFlywheel, kdFlywheel, flywheelMotor.getCurrentPosition(), FlywheelRPM);
            else
                FlywheelRPM = flywheelMotor.getCurrentPosition();
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



    //gripper methods

    boolean grabState = false;

    public boolean toggleGripper() {
        grabState = !grabState;
        if(grabState)
            servoGripper.setPosition(servoNormalize(1500)); //open
        else
            servoGripper.setPosition(servoNormalize(899)); //closed
        return true;
    }



    //flywheel methods



    public void movePIDExtend(double Kp, double Ki, double Kd, double currentTicks, double targetTicks) {

        //initialization of the PID calculator's output range, target value and multipliers
        flyWheelPID.setOutputRange(-FlywheelPwr, FlywheelPwr);
        flyWheelPID.setPID(Kp, Ki, Kd);
        flyWheelPID.setSetpoint(targetTicks);
        flyWheelPID.enable();

        //initialization of the PID calculator's input range and current value
        //extendPID.setInputRange(0, 360);
        //extendPID.setContinuous();
        flyWheelPID.setInput(currentTicks);

        //calculates the correction to apply
        FlywheelCorrection = flyWheelPID.performPID();

        //performs the extension with the correction applied
        flywheelMotor.setPower(FlywheelCorrection);
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
        setElbowTargetPos((int) (ticksPerDegree* angleDegrees));
        return true;
    }

    public int getElbowTargetPos(){
        return elbowPos;
    }
    public int getElbowCurrentPos(){
        return elbow.getCurrentPosition();
    }
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

