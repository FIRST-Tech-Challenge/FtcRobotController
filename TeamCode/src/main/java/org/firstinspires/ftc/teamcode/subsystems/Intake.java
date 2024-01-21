package org.firstinspires.ftc.teamcode.subsystems;
import android.util.Log;

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
    private double baseposl_yield = 0.36+0.25;//+0.4
    private double baseposr_yield = 0.615-0.25;//-0.4
    private double outtakeposL = 0.814 + 0.033; //0.804
    private double outtakeposR = 0.1403 - 0.033; //0.1503
    //placeholder outtake position, may change depending on outtake
    private double intakeposL = 0.129+0.037;//+0.037
    private double intakeposR = 0.8595-0.037;//-0.037
    private double lowerlimitL = 0.814;
    private double lowerlimitR = 0.13;
    private double upperlimitL = 0.09;
    private double upperlimitR = 0.9;

    //State Machine
    public int intakeState = 0; //0: basePos, motor=0, 1: intakePos, motor=1, 2: outtakePos, motor=1
    private double motorDelayAfterOut = 2000; // in ms. after arm moves to outtake, delay to stop motor
    private double motorDelayForAuto = 1300; // in ms. after arm moves to outtake, delay to stop motor
    private double motorDelayForAutoOutput = 800;

    private double outtakeStartTime = 0;
    private double intakeReverseStartTime = 0;
    private double motorSweepPwr = -1.0;
    private double autoOutputPwr = -0.4;

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
        //Log.v("intake", "to intake pos");
    }
    public void toOuttakePos(){
        intakeServoL.setPosition(outtakeposL);
        intakeServoR.setPosition(outtakeposR);
    }

    public void toBasePos(){
        intakeServoL.setPosition(baseposl);
        intakeServoR.setPosition(baseposr);
    }

    public void toBasePosYield(){
        intakeServoL.setPosition(baseposl_yield);
        intakeServoR.setPosition(baseposr_yield);
    }

    public void moveArm(double d){
        double targetPosR = intakeServoR.getPosition()+(0.01*d*syncFactor);
        if(targetPosR>lowerlimitR&&targetPosR<upperlimitR) {
            intakeServoL.setPosition(intakeServoL.getPosition() + (0.01 * -d)); //2 degrees??
            intakeServoR.setPosition(intakeServoR.getPosition() + (0.01 * d * syncFactor));
        }
    }
    public void moveArmNoLimit(double d){
        intakeServoL.setPosition(intakeServoL.getPosition() + (0.04 * -d)); //2 degrees??
        intakeServoR.setPosition(intakeServoR.getPosition() + (0.04 * d * syncFactor));
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
            toBasePosYield();
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
        } else if (intakeState == 11) {//Start output pre-load pixel
            toIntakePos();
            //intakeMotor.setPower(this.autoOutputPwr);
            intakeState = 12;
            outtakeStartTime = System.currentTimeMillis();
        } else if (intakeState == 12) { //wait for intake to intakePos
            long time = System.currentTimeMillis();
            if (time - outtakeStartTime >= this.motorDelayForAuto) {
                intakeState = 13;
                intakeMotor.setPower(-1*this.autoOutputPwr);
                outtakeStartTime = System.currentTimeMillis();
            }
        } else if (intakeState == 13) {//output
            if (System.currentTimeMillis() - outtakeStartTime >= this.motorDelayForAutoOutput) {
                intakeState = 0;
                intakeMotor.setPower(0);
                toBasePosYield();
            }
        } else if(intakeState == 21) { // Start reverse intake roller motor
            //intakeReverseStartTime = System.currentTimeMillis();
            intakeMotor.setPower(1);
            //intakeState = 8;
        } else if(intakeState == 22){
            intakeMotor.setPower(-1);
        }  else if (intakeState == 31) {//Start output pre-load pixel. AUTO ONLY, DO NOT CHANGE
            toIntakePos();
            //intakeMotor.setPower(this.autoOutputPwr);
            intakeState = 32;
            outtakeStartTime = System.currentTimeMillis();
        } else if (intakeState == 32) { //wait for intake to intakePos. AUTO ONLY, DO NOT CHANGE
            long time = System.currentTimeMillis();
            if (time - outtakeStartTime >= this.motorDelayForAuto) {
                intakeState = 33;
                intakeMotor.setPower(this.autoOutputPwr);
                outtakeStartTime = System.currentTimeMillis();
            }
        } else if (intakeState == 33) {//output, AUTO ONLY, DO NOT CHANGE
            if (System.currentTimeMillis() - outtakeStartTime >= this.motorDelayForAutoOutput) {
                intakeState = 0;
                intakeMotor.setPower(0);
                toBasePosYield();
            }
        }


        //intakeMotor.setPower(motorPosition);
    }
}
