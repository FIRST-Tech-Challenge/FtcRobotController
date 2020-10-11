package org.firstinspires.ftc.teamcode.robots.kraken;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by tycho on 10/15/2017.
 */

public class GlyphSystem {

    DcMotor motorLift = null;
    DcMotor motorLeft = null;
    DcMotor motorRight = null;
    Servo servoGrip = null;
    Servo servoBeltLeft = null;
    Servo servoBeltRight = null;
    Servo servoPhone = null;

    private int liftMax = 4000;
    private int liftStack = 2500; //stacking height
    private int liftMin = 50;
    private int liftAuto = 500;
    private int liftAuto2 = 1500;
    private int beltOn = 2000;
    private int beltOff = 1500;
    private int phoneUp = 900; //1250
    private int phoneDown = 2105; //1850
    private int liftPlanck = 450; //smallest distance to increment lift by when using runToPosition

    boolean gripOpen = false;
    int gripOpenPos = 1544; //1400
    //int gripClosedPos = 1900;
    int gripClosedPos=1735;
    int gripTightPos=1700;
    int gripWideOpenPos = 1381;

    public GlyphSystem(DcMotor motorLift, Servo servoGrip, DcMotor motorLeft, DcMotor motorRight, Servo servoBeltLeft, Servo servoBeltRight, Servo servoPhone){
        this.motorLift = motorLift;
        this.servoGrip = servoGrip;
        this.motorLeft = motorLeft;
        this.motorRight = motorRight;
        this.servoBeltLeft = servoBeltLeft;
        this.servoBeltRight = servoBeltRight;
        this.servoPhone = servoPhone;
        this.servoBeltLeft.setPosition(servoNormalize(beltOff));
        this.servoBeltRight.setPosition(servoNormalize(beltOff));
        this.servoGrip.setPosition(servoNormalize(gripClosedPos));
    }

    public void collect(){
        motorLeft.setPower(.90);
        motorRight.setPower(-.90);
    }

    public void hold(){

        motorLeft.setPower(0);
        motorRight.setPower(0);
    }

    public void toggleGrip(){
        if (gripOpen) {
            gripOpen = false;
            servoGrip.setPosition(servoNormalize(gripClosedPos));
        }
        else {
            gripOpen = true;
            servoGrip.setPosition(servoNormalize(gripWideOpenPos));
        }
    }
    public void closeGrip() {
        gripOpen = false;
        servoGrip.setPosition(servoNormalize(gripClosedPos));
    }

    public void closeGripTight() {
        gripOpen = false;
        servoGrip.setPosition(servoNormalize(gripTightPos));
    }

    public void releaseGrip() {
        gripOpen = true;
        servoGrip.setPosition(servoNormalize(gripOpenPos));
    }

    public void wideOpenGrip() {
        gripOpen = true;
        servoGrip.setPosition(servoNormalize(gripWideOpenPos));
    }

    public void tiltPhoneUp(){
        servoPhone.setPosition(servoNormalize(phoneUp));
    }

    public void tiltPhoneDown(){
        servoPhone.setPosition(servoNormalize(phoneDown));
    }

    public void togglePhoneTilt() {
        if(servoPhone.getPosition() == servoNormalize(phoneUp)){
            tiltPhoneDown();
        }
        else{
            tiltPhoneUp();
        }
    }

    public void liftBelt () {
        servoBeltLeft.setPosition(servoNormalize(beltOn));
        servoBeltRight.setPosition(servoNormalize(beltOn));
    }


    public void stopBelt () {
        servoBeltLeft.setPosition(servoNormalize(beltOff));
        servoBeltRight.setPosition(servoNormalize(beltOff));
    }

    public void toggleBelt () {
        if(servoBeltRight.getPosition() == servoNormalize(beltOff)){
            servoBeltLeft.setPosition(servoNormalize(beltOn));
            servoBeltRight.setPosition(servoNormalize(beltOn));
            collect();

        }
        else{
            servoBeltLeft.setPosition(servoNormalize(beltOff));
            servoBeltRight.setPosition(servoNormalize(beltOff));
            hold();
        }
    }

    public void setMotorLeft (double pwr){
        motorLeft.setPower(pwr);
    }

    public void setMotorRight (double pwr){
        motorRight.setPower(pwr);
    }

    public void stopLift(){

        motorLift.setPower(0);
        stopBelt();
        setMotorLeft(0);
        setMotorRight(0);

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

    public void goLiftAuto2() {

        motorLift.setTargetPosition(liftAuto2);
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
