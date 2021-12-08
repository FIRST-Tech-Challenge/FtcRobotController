package org.firstinspires.ftc.teamcode.robots.reach;

//written by Cooper Clem, 2021

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.robots.UGBot.utils.Constants;
import org.firstinspires.ftc.teamcode.util.PIDController;
import org.firstinspires.ftc.teamcode.util.RateController;

import static org.firstinspires.ftc.teamcode.util.utilMethods.between360Clockwise;
import static org.firstinspires.ftc.teamcode.util.utilMethods.diffAngle2;
import static org.firstinspires.ftc.teamcode.util.utilMethods.nextCardinal;
import static org.firstinspires.ftc.teamcode.util.utilMethods.wrap360;
import static org.firstinspires.ftc.teamcode.util.utilMethods.wrapAngle;
import static org.firstinspires.ftc.teamcode.util.utilMethods.wrapAngleMinus;

@Config
public class Turret{
    //motor
    private  DcMotor motor = null;
    private boolean active = true;

    //PID
    PIDController turretPID;
    public static double kpTurret = 0.03; //proportional constant multiplier goodish
    public static  double kiTurret = 0; //integral constant multiplier
    public static  double kdTurret= .05; //derivative constant multiplier
    double correction = 0.00; //correction to apply to turret motor

    double turretHeading;
    boolean initialized = false;
    private double turretTargetHeading = 0.0;

    public static double TURRET_TICKS_PER_DEGREE = 15.320535022020206; //determine with a calibration run

    public Turret(DcMotor motor) {
        this.motor = motor;
        turretTargetHeading = 0.0;
        turretPID = new PIDController(0,0,0);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void update(){
        turretHeading = motor.getCurrentPosition() / (1 / TURRET_TICKS_PER_DEGREE);

        if(active) {
            movePIDTurret(kpTurret, kiTurret, kdTurret, turretHeading, turretTargetHeading);
        }
        else
            motor.setPower(0);
    }

    public boolean isActive(){
        return active;
    }

    public void setActive(boolean active){this.active = active;}

    public boolean rotateCardinalTurret(boolean right){

        setTurretAngle(nextCardinal(getHeading(),right,10));

        return true;
    }

    public void stopAll() {
        setPower(0);
        active = false;
    }

    public boolean setTurretAngle(double angle){
        turretTargetHeading=wrap360(angle);
        return isTurretNearTarget();
    }

    public boolean isTurretNearTarget(){
        return between360Clockwise(getHeading(), getTargetHeading() - Constants.TURRET_TOLERANCE, getTargetHeading() + Constants.TURRET_TOLERANCE);
    }

    private void setPower(double pwr){
        motor.setPower(pwr);
    }

    double turnError = 0;
    public void movePIDTurret(double Kp, double Ki, double Kd, double currentAngle, double targetAngle) {
        //initialization of the PID calculator's output range, target value and multipliers
        turretPID.setOutputRange(-.69, .69); //this is funny
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

    public double getHeading(){
            return turretHeading;
    }

    public double getTargetHeading(){
            return turretTargetHeading;
    }

    public double getCorrection(){return correction;}
    public double getMotorPwr(){return motor.getPower();}
}

