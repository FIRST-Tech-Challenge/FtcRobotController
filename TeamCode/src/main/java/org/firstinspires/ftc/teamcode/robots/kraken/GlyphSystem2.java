package org.firstinspires.ftc.teamcode.robots.kraken;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/**
 * Created by tycho on 10/15/2017.
 */

public class GlyphSystem2 {

    DcMotor motorLift = null;
    DcMotor motorLeft = null;
    DcMotor motorRight = null;
    Servo servoGripRight = null;
    Servo servoGripLeft = null;
    Servo servoGripBottom = null;
    Servo servoBeltLeft = null;
    Servo servoBeltRight = null;
    Servo servoPhone = null;


    private int liftMax = 4000;
    private int liftStack = 2500; //stacking height
    private int liftMin = 50;
    private int liftAuto = 500;
    private int liftAuto2 = 1500;
    public int beltOn = 2000;
    private int beltOff = 1500;
    private int beltReverse = 1000;
    private int phoneUp = 1700; //900
    private int phoneDown = 1150; //2105
    private int phoneMax = 2100;
    private double phonePWMPerDegree = 8.889;
    private int liftPlanck = 450; //smallest distance to increment lift by when using runToPosition

    public int liftDeposit = 1065;
    public int liftVerticalDeposit =  1160;
    public int liftRecoveryPos = 560;
    public int liftCollect = -3743;

    public int liftFlatUpper = 450;
    public int liftFlatLower = -2426;

    private int liftStage = 0;
    private long liftTimer = 0;

    private double offsetHeading;
    private double offsetPitch;
    private double offsetRoll;

    public double pitch = 0;
    public double yaw = 0;
    public double roll = 0;

    BNO055IMU imu;
    Orientation imuAngles;


//    private int liftDown =

    //;top two
    boolean gripOpen=false;
    boolean bottomGripOpen = false;
    int gripOpenPos = 1350; //1544
    //int gripClosedPos = 1900;
    int gripClosedPos=1200; //1735
    int gripTightPos= 1100; //1700
    int gripWideOpenPos = 1450; //1381

    //bottom servo
    int gripBWideOpenPos=1800;
    int gripBOpenPos = 1500;
    int gripBClosedPos = 1270;
    int gripBTightPos=1150;


    public GlyphSystem2(DcMotor motorLift, Servo servoGripRight, Servo servoGripLeft, Servo servoGripBottom, DcMotor motorLeft, DcMotor motorRight, Servo servoBeltLeft, Servo servoBeltRight, Servo servoPhone, BNO055IMU imu){

        this.imu = imu;
        this.motorLift = motorLift;
        this.servoGripRight = servoGripRight;
        this.servoGripLeft = servoGripLeft;
        this.servoGripBottom=servoGripBottom;
        this.motorLeft = motorLeft;
        this.motorRight = motorRight;
        this.servoBeltLeft = servoBeltLeft;
        this.servoBeltRight = servoBeltRight;
        this.servoPhone = servoPhone;
        this.servoBeltLeft.setPosition(servoNormalize(beltOff));
        this.servoBeltRight.setPosition(servoNormalize(beltOff));
        this.servoGripRight.setPosition(servoNormalize(gripClosedPos));

    }

    public void collect(){
        motorLeft.setPower(1);
        motorRight.setPower(-1);
    }

    public void tiltPhoneMax(){
        servoPhone.setPosition(servoNormalize(phoneMax));
    }

    public void hold(){

        motorLeft.setPower(0);
        motorRight.setPower(0);
    }

    public int maintainPhoneTilt(){
        double pos = phoneDown + (360 - roll)*phonePWMPerDegree;
        if(pos>phoneUp) pos = phoneUp;
        if(pos<phoneDown) pos = phoneDown;
        servoPhone.setPosition(servoNormalize((int)pos));
        return (int)pos;
    }

    public void toggleGrip(){
        if (gripOpen) {
            gripOpen = false;
            bottomGripOpen = false;

            servoGripRight.setPosition(servoNormalize(gripClosedPos));
            servoGripLeft.setPosition(servoNormalize(gripClosedPos));
            servoGripBottom.setPosition(servoNormalize(gripBClosedPos));
        }
        else {
            gripOpen = true;
            bottomGripOpen = true;

            servoGripLeft.setPosition(servoNormalize(gripWideOpenPos));
            servoGripRight.setPosition(servoNormalize(gripWideOpenPos));
            servoGripBottom.setPosition(servoNormalize(gripBWideOpenPos));
        }
    }

    public void toggleBottomGrip(){
        if(bottomGripOpen){
            bottomGripOpen = false;

            servoGripBottom.setPosition(servoNormalize(gripBClosedPos));
        }
        else{
            bottomGripOpen = true;
            servoGripBottom.setPosition(servoNormalize(gripBOpenPos));
        }
    }

    public void closeGrip() {
        gripOpen = false;
        servoGripLeft.setPosition(servoNormalize(gripClosedPos));
        servoGripRight.setPosition(servoNormalize(gripClosedPos));
        servoGripBottom.setPosition(servoNormalize(gripBClosedPos));
    }

    public void closeGripTight() {
        gripOpen = false;
        servoGripLeft.setPosition(servoNormalize(gripTightPos));
        servoGripRight.setPosition(servoNormalize(gripTightPos));
        servoGripBottom.setPosition(servoNormalize(gripBTightPos));
    }

    public void releaseGrip() {
        gripOpen = true;
        servoGripLeft.setPosition(servoNormalize(gripOpenPos));
        servoGripRight.setPosition(servoNormalize(gripOpenPos));
        servoGripBottom.setPosition(servoNormalize(gripBOpenPos));
    }

    public void wideOpenGrip() {
        gripOpen = true;
        servoGripLeft.setPosition(servoNormalize(gripWideOpenPos));
        servoGripRight.setPosition(servoNormalize(gripWideOpenPos));
        servoGripBottom.setPosition(servoNormalize(gripBWideOpenPos));
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

    public boolean goLiftCollect(){
        switch (liftStage){
            case 0:
                tiltPhoneUp();
                motorLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                motorLift.setPower(.6);
                liftStage++;
                break;
            case 1:
                if(motorLift.getCurrentPosition() > liftDeposit - 200) {
                    motorLift.setPower(.3);
                    motorLift.setTargetPosition(liftRecoveryPos);
                    liftTimer = futureTime(1.5f);
                }
                if(System.nanoTime() > liftTimer) {
                    liftStage++;
                    liftTimer = futureTime(5.0f);
                }
                break;
            case 2:
                if(motorLift.getCurrentPosition() > liftFlatUpper || motorLift.getCurrentPosition() < liftFlatLower){
                    motorLift.setPower(.6);
                }
                else
                    motorLift.setPower(1);
                motorLift.setTargetPosition(liftCollect);
                if(motorLift.getCurrentPosition() > liftCollect - 50 && motorLift.getCurrentPosition() < liftCollect + 50){
                    liftStage = 0;
                    return true;
                }
                break;
            case 3:
                break;
        }
        if(motorLift.getMode() != DcMotor.RunMode.RUN_TO_POSITION){
            liftStage = 0;
            return true;
        }
        return false;
    }


    public void gripBOpen(){
        servoGripBottom.setPosition(servoNormalize(gripBOpenPos));
    }

    public void gripBClose(){
        servoGripBottom.setPosition(servoNormalize(gripBClosedPos));
    }

    public boolean goLiftDeposit(){
        switch (liftStage){
            case 0:
                tiltPhoneUp();
                motorLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                liftStage++;
                break;
            case 1:
                if(motorLift.getCurrentPosition() > liftFlatUpper || motorLift.getCurrentPosition() < liftFlatLower){
                    motorLift.setPower(.6);
                }
                else
                    motorLift.setPower(1);
                motorLift.setTargetPosition(liftDeposit);
                liftTimer = futureTime(5f);
                liftStage++;
                break;
            case 2:
                if(motorLift.getCurrentPosition() > liftDeposit - 10 || motorLift.getCurrentPosition() > liftDeposit - 10){
                    liftStage = 0;
                    return true;
                }
                break;
        }
        if(motorLift.getMode() != DcMotor.RunMode.RUN_TO_POSITION){
            liftStage = 0;
            return true;
        }
        return false;
    }

    public boolean goLiftVerticalDeposit(){
        switch (liftStage){
            case 0:
                tiltPhoneUp();
                motorLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                motorLift.setPower(.6);
                liftStage++;
                break;
            case 1:
                if(motorLift.getCurrentPosition() > liftFlatUpper || motorLift.getCurrentPosition() < liftFlatLower){
                    motorLift.setPower(.6);
                }
                else
                    motorLift.setPower(1);
                motorLift.setTargetPosition(liftVerticalDeposit);
//                liftTimer = futureTime(5f);
                liftStage++;
                break;
            case 2:
                if(motorLift.getCurrentPosition() > liftFlatUpper || motorLift.getCurrentPosition() < liftFlatLower){
                    motorLift.setPower(.6);
                }
                else
                    motorLift.setPower(1);
                if(motorLift.getCurrentPosition() > liftVerticalDeposit - 10 || motorLift.getCurrentPosition() > liftVerticalDeposit - 10){
                    liftStage = 0;
                    return true;
                }
                break;
        }
        if(motorLift.getMode() != DcMotor.RunMode.RUN_TO_POSITION){
            liftStage = 0;
            return true;
        }
        return false;
    }

    public boolean goHome(){
        switch (liftStage){
            case 0:
                tiltPhoneUp();
                motorLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorLift.setPower(.6);
                liftStage++;
                break;
            case 1:
                if(motorLift.getCurrentPosition() > liftDeposit - 200) {
//                    motorLift.setPower(.2);
                    motorLift.setTargetPosition(liftRecoveryPos);
                    liftTimer = futureTime(1.5f);
                }
                if(System.nanoTime() > liftTimer) {
                    liftStage++;
                    liftTimer = futureTime(3.0f);
                }
                break;
            case 2:
//                motorLift.setPower(.6);
                if(motorLift.getCurrentPosition() > liftFlatUpper || motorLift.getCurrentPosition() < liftFlatLower){
                    motorLift.setPower(.2);
                }
                else
                    motorLift.setPower(1);
                motorLift.setTargetPosition(0);
                if(System.nanoTime() > liftTimer){
                    liftStage = 0;
                    return true;
                }
                break;
            case 3:
                break;
        }
        if(motorLift.getMode() != DcMotor.RunMode.RUN_TO_POSITION){
            liftStage = 0;
            return true;
        }
        return false;

    }


    public void resetLiftStage(){
        liftStage = 0;
    }

    long futureTime(float seconds){
        return System.nanoTime() + (long) (seconds * 1e9);
    }


    public void liftBelt () {
        if(motorLift.getCurrentPosition() > liftDeposit - 50){
            servoBeltLeft.setPosition(servoNormalize(beltReverse));
            servoBeltRight.setPosition(servoNormalize(beltReverse));
        }
        else {
            servoBeltLeft.setPosition(servoNormalize(beltOn));
            servoBeltRight.setPosition(servoNormalize(beltOn));
        }
    }


    public void stopInternalLift() {
        servoBeltLeft.setPosition(servoNormalize(beltOff));
        servoBeltRight.setPosition(servoNormalize(beltOff));
    }

    public void toggleBelt (boolean reverse) {
        if(servoBeltRight.getPosition() == servoNormalize(beltOff)){
            if(reverse){
                servoBeltLeft.setPosition(servoNormalize(beltReverse));
                servoBeltRight.setPosition(servoNormalize(beltReverse));
            }
            else{
                servoBeltLeft.setPosition(servoNormalize(beltOn));
                servoBeltRight.setPosition(servoNormalize(beltOn));
            }

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

//    public void raiseLift2(){
//        if (motorLift.getCurrentPosition() < liftMax && motorLift.getTargetPosition() < liftMax) {
//            motorLift.setTargetPosition((int) Math.min(motorLift.getCurrentPosition() + liftPlanck, liftMax));
//            motorLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            motorLift.setPower(1);
//        }
//    }
//    public void lowerLift2() {
//        if (motorLift.getCurrentPosition() > liftMin && motorLift.getTargetPosition() > liftMin) {
//            motorLift.setTargetPosition((int) Math.max(motorLift.getCurrentPosition() - liftPlanck, liftMin));
//            motorLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            motorLift.setPower(.8);
//        }
//    }

    public void raiseLift2 (){
        motorLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        if(motorLift.getCurrentPosition() > liftFlatUpper || motorLift.getCurrentPosition() < liftFlatLower){
            motorLift.setPower(.3);
        }
        else
            motorLift.setPower(1);
    }

    public void lowerLift2 (){
        motorLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        if(motorLift.getCurrentPosition() > liftFlatUpper || motorLift.getCurrentPosition() < liftFlatLower){
            motorLift.setPower(-.6);
        }
        else
            motorLift.setPower(-1);
    }

    public void resetLift(){
        motorLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void stopBelt() {
        if(motorLift.getMode() == DcMotor.RunMode.RUN_USING_ENCODER) motorLift.setPower(0);
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

    public void initIMU (){

        imuAngles= imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);

        offsetHeading = wrapAngleMinus(yaw, imuAngles.firstAngle);
        offsetRoll = wrapAngleMinus(imuAngles.secondAngle, roll);
        offsetPitch = wrapAngleMinus(imuAngles.thirdAngle, pitch);

    }

    public void update (){

        imuAngles= imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);

        yaw = wrapAngle(imuAngles.firstAngle, offsetHeading);
        pitch = wrapAngle(imuAngles.thirdAngle, offsetPitch);
        roll = wrapAngle(imuAngles.secondAngle, offsetRoll);
    }

    public double wrapAngle(double angle1, double angle2){
        return (angle1 + angle2) % 360;
    }
    public double wrapAngleMinus(double angle1, double angle2){
        return 360-((angle1 + angle2) % 360);
    }

}
