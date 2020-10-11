package org.firstinspires.ftc.teamcode.robots.kraken;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by tycho on 10/15/2017.
 */

public class PickAndPlace {

    DcMotor motorLift = null;
    Servo servoGrip = null;
    Servo servoLeft = null;
    Servo servoRight = null;

    private int liftMax = 4000;
    private int liftStack = 2500; //stacking height
    private int liftMin = 50;
    private int liftAuto = 500;
    private int liftPlanck = 450; //smallest distance to increment lift by when using runToPosition

    boolean gripOpen = false;
    int gripOpenPos = 1000;
    int gripClosedPos = 2110;

    public PickAndPlace(DcMotor motorLift, Servo servoGrip, Servo servoLeft, Servo servoRight){
        this.motorLift = motorLift;
        this.servoGrip = servoGrip;
        this.servoLeft = servoLeft;
        this.servoRight = servoRight;
    }

    public void toggleGrip(){
        if (gripOpen) {
            gripOpen = false;
            servoGrip.setPosition(servoNormalize(gripClosedPos));
        }
        else {
            gripOpen = true;
            servoGrip.setPosition(servoNormalize(gripOpenPos));
        }
    }
    public void closeGrip() {
        gripOpen = false;
        servoGrip.setPosition(servoNormalize(gripClosedPos));
    }

    public void releaseGrip() {
        gripOpen = true;
        servoGrip.setPosition(servoNormalize(gripOpenPos));
    }

    public void setServoLeft (int pulse){
        servoLeft.setPosition(servoNormalize(pulse));
    }

    public void setServoRight (int pulse){
        servoRight.setPosition(servoNormalize(pulse));
    }

    public void stopLift(){
        motorLift.setPower(0);
    }

    public void raiseLift(){
        if(motorLift.getCurrentPosition() < liftMax) motorLift.setPower(.5);
        else motorLift.setPower(0);
    }
    public void lowerLift(){
        if(motorLift.getCurrentPosition() > liftMin) motorLift.setPower(-.5);
        else motorLift.setPower(0);
    }

    public void raiseLift2(){
        if (motorLift.getCurrentPosition() < liftMax && motorLift.getTargetPosition() < liftMax) {
            motorLift.setTargetPosition((int) Math.min(motorLift.getCurrentPosition() + liftPlanck, liftMax));
            motorLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorLift.setPower(1);
        }
    }
    public void lowerLift2() {
        if (motorLift.getCurrentPosition() > liftMin && motorLift.getTargetPosition() > liftMin) {
            motorLift.setTargetPosition((int) Math.max(motorLift.getCurrentPosition() - liftPlanck, liftMin));
            motorLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorLift.setPower(.8);
        }
    }
    public void goLiftMax() {

            motorLift.setTargetPosition(liftMax);
            motorLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorLift.setPower(1);

    }

    public void goLiftAuto() {

        motorLift.setTargetPosition(liftAuto);
        motorLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorLift.setPower(1);

    }



    public void goLiftMin() {

        motorLift.setTargetPosition(liftMin);
        motorLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorLift.setPower(1);

    }

    public void goLiftStack() {

        motorLift.setTargetPosition(liftStack);
        motorLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorLift.setPower(1);

    }

    public int getMotorLiftPosition(){
        return motorLift.getCurrentPosition();
    }

    public static double servoNormalize(int pulse){
        double normalized = (double)pulse;
        return (normalized - 750.0) / 1500.0; //convert mr servo controller pulse width to double on _0 - 1 scale
    }

}
