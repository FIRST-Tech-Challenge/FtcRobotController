package org.firstinspires.ftc.teamcode.subsystems;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.Subsystem;


public class Intake implements Subsystem {
    //Hardware: 1 motor, 2 servo
    double syncFactor = 1.05;
    private DcMotorEx intakeMotor;
    private double motorPosition = 0; private Servo intakeServoL;
    private Servo intakeServoR;
    private double baseposl = 0.217;
    private double baseposr = 0.76715;
    private double intakeposL = 0.814 - 0.01;
    private double intakeposR = 0.1403 + 0.01;
    //placeholder outtake position, may change depending on outtake
    private double outtakeposL = 0.129;
    private double outtakeposR = 0.8595;
    private double lowerlimitL = 0.814;
    private double lowerlimitR = 0.13;
    private double upperlimitL = 0.09;
    private double upperlimitR = 0.9;

    //State Machine
    public int intakeState = 0; //0: basePos, motor=0, 1: intakePos, motor=1, 2: outtakePos, motor=1
    private double motorDelayAfterOut = 1000; // in ms. after arm moves to outtake, delay to stop motor
    private double motorDelayForAuto = 200; // in ms. after arm moves to outtake, delay to stop motor

    private double outtakeStartTime = 0;
    private double motorSweepPwr = 1.0;

    public Intake(Robot robot) {
        intakeMotor = robot.getMotor("intakeMotor");
        intakeServoL = robot.getServo("intakeServoL");
        intakeServoR = robot.getServo("intakeServoR");
        toBasePos();
    }
    public void reset(){
        intakeServoL.setPosition(baseposl);
        intakeServoR.setPosition(baseposr);
    }
    public void toIntakePos(){
        intakeServoR.setPosition(intakeposR);
        intakeServoL.setPosition(intakeposL);
    }
    public void toOuttakePos(){
        intakeServoL.setPosition(outtakeposL);
        intakeServoR.setPosition(outtakeposR);
    }

    public void toBasePos(){
        intakeServoL.setPosition(baseposl);
        intakeServoR.setPosition(baseposr);
    }


    public void moveArm(double d){
        double targetPosR = intakeServoR.getPosition()+(0.01*d*syncFactor);
        if(targetPosR>lowerlimitR&&targetPosR<upperlimitR) {
            intakeServoL.setPosition(intakeServoL.getPosition() + (0.01 * -d)); //2 degrees??
            intakeServoR.setPosition(intakeServoR.getPosition() + (0.01 * d * syncFactor));
        }
    }
    public double getRightServoPos() {
        return intakeServoR.getPosition();
    }
    public double getLeftServoPos(){
        return intakeServoL.getPosition();
    }


    public void setIntakeState(int state) {
        this.intakeState = state;
    }
    public void setPower(double power) {
        this.motorPosition = -power;
        // set encode to new position
    }

    public double getPower() {
        return intakeMotor.getPower();
    }


    @Override
    public void update(TelemetryPacket packet) {

        if (intakeState == 0) {//Base, idle
            toBasePos();
            intakeMotor.setPower(0);
        } else if (intakeState == 1) {//Intake
            toIntakePos();
            intakeMotor.setPower(this.motorSweepPwr);
        } else if (intakeState == 2) {//Start outtake
            toOuttakePos();
            intakeMotor.setPower(this.motorSweepPwr);
            intakeState = 3;
            outtakeStartTime = System.currentTimeMillis();
        } else if (intakeState == 3) {//Done outtake
            long time = System.currentTimeMillis();
            if(time - outtakeStartTime >= this.motorDelayAfterOut){
                intakeState = 0;
            }
        } else if (intakeState == 4) {//Start output pre-load pixel
            toIntakePos();
            intakeMotor.setPower(-this.motorSweepPwr);
            intakeState = 5;
            outtakeStartTime = System.currentTimeMillis();
        } else if (intakeState == 5) {
            long time = System.currentTimeMillis();
            if(time - outtakeStartTime >= this.motorDelayForAuto){
                intakeState = 0;
                intakeMotor.setPower(0);
                toBasePos();
            }
        }
        //intakeMotor.setPower(motorPosition);
    }
}
