package org.firstinspires.ftc.teamcode;
//Test
//By Ethan Clawsie and Aman Sulaiman, 2021-2022 Freight Frenzy

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.TouchSensor;


public class piRhosHardware {
    public DcMotor frontLeft, backLeft, frontRight, backRight, cascadeMotor1, cascadeMotor2, slider, carousel;
    public TouchSensor touchRight, touchLeft;
    public Servo bucket;
    public CRServo intake1, intake2;
    HardwareMap hwMap;
    public ElapsedTime timer = new ElapsedTime();
    public void initTeleOpIMU(HardwareMap hwMap) {
        this.hwMap = hwMap;
        timer.reset();
        backLeft = hwMap.dcMotor.get("back_left");
        backRight = hwMap.dcMotor.get("back_right");
        frontLeft = hwMap.dcMotor.get("front_left");
        frontRight = hwMap.dcMotor.get("front_right");
        carousel = hwMap.dcMotor.get("carousel");
        cascadeMotor1 = hwMap.dcMotor.get("cascadeMotorRight");
        cascadeMotor2 = hwMap.dcMotor.get("cascadeMotorLeft");
        //cascadeTouch = hwMap.touchSensor.get("cascade_touch");
        bucket = hwMap.servo.get("bucket");
        //intake1 = hwMap.crservo.get("intake1");
        //intake2 = hwMap.crservo.get("intake2");
        //slider = hwMap.dcMotor.get("slider");
        touchRight = hwMap.touchSensor.get("touch_right");
        touchLeft = hwMap.touchSensor.get("touch_left");
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        boolean go = true;

        //bucket.setPosition(.9);
    }


    public void setPowerOfAllMotorsToForTime(double power, double time)
    {
        timer.reset();
        while(timer.seconds() <= time){
            backLeft.setPower(power);
            backRight.setPower(power);
            frontLeft.setPower(power);
            frontRight.setPower(power);
        }
        backLeft.setPower(0);
        backRight.setPower(0);
        frontLeft.setPower(0);
        frontRight.setPower(0);
    }
    //Strafes right for certain amount of time
    public void strafeRightForTime(double power, double time)
    {
        timer.reset();
        while(timer.seconds() <= time){
            backLeft.setPower(-power);
            backRight.setPower(power);
            frontLeft.setPower(power);
            frontRight.setPower(-power);
        }
        backLeft.setPower(0);
        backRight.setPower(0);
        frontLeft.setPower(0);
        frontRight.setPower(0);
    }
    //strafes left for certain amount of time
    public void strafeLeftForTime(double power, double time)
    {
        timer.reset();
        while(timer.seconds() <= time){
            backLeft.setPower(power);
            backRight.setPower(-power);
            frontLeft.setPower(-power);
            frontRight.setPower(power);
        }
        backLeft.setPower(0);
        backRight.setPower(0);
        frontLeft.setPower(0);
        frontRight.setPower(0);
    }
    //turns left for certain amount of time
    public void turnRightForTime(double power, double time)
    {
        timer.reset();
        while(timer.seconds() <= time){
            backLeft.setPower(power);
            backRight.setPower(-power);
            frontLeft.setPower(power);
            frontRight.setPower(-power);
        }
        backLeft.setPower(0);
        backRight.setPower(0);
        frontLeft.setPower(0);
        frontRight.setPower(0);
    }
    //turns right for certain amount of time
    public void turnLeftForTime(double power, double time)
    {
        timer.reset();
        while(timer.seconds() <= time){
            backLeft.setPower(-power);
            backRight.setPower(power);
            frontLeft.setPower(-power);
            frontRight.setPower(power);
        }
        backLeft.setPower(0);
        backRight.setPower(0);
        frontLeft.setPower(0);
        frontRight.setPower(0);
    }
    //gives you Elapsed time
    public double getTime(){
        return timer.time();
    }
    public void setPowerOfAllMotorsTo(double power)
    {
        backLeft.setPower(power);
        backRight.setPower(power);
        frontLeft.setPower(power);
        frontRight.setPower(power);
    }
    public void strafeLeft(double power) {

        backLeft.setPower(power);
        backRight.setPower(-power);
        frontLeft.setPower(-power);
        frontRight.setPower(power);
    }

    public void strafeRight(double power) {

        backLeft.setPower(-power);
        backRight.setPower(power);
        frontLeft.setPower(power);
        frontRight.setPower(-power);
    }



    /*public void slideForTime(double power, double time)
    {
        timer.reset();
        while(timer.seconds() <= time){
            slider.setPower(power);
        }
        slider.setPower(0);
    }*/
    public void cascadeupfortime(double power, double time)
    {
        timer.reset();
        while(timer.seconds() <= time){
            cascadeMotor1.setPower(power);
            cascadeMotor2.setPower(power);
        }
        cascadeMotor1.setPower(0.1);
        cascadeMotor2.setPower(0.1);
    }
    /*public void intakeForTime(double power, double time) {
        timer.reset();
        while (timer.seconds() <= time) {
            intake1.setPower(power);
            intake2.setPower(-power);
        }

    }*/
    public void carouselpowerfortime(double power, double time) {
        timer.reset();
        while (timer.seconds() <= time) {
            carousel.setPower(power);
        }
        carousel.setPower(0);
    }
    public void movedist(int dist) {
        double startticks = frontLeft.getCurrentPosition();
        setPowerOfAllMotorsTo(0.5);
        while (1 == 1) {
            double getticks = frontLeft.getCurrentPosition();
            double distance = (getticks - startticks) * (11.8672 / 537.6);
            if (distance >= dist) {
                setPowerOfAllMotorsTo(0);
                break;
            }
        }
    }
    public void movebackdist(int dist) {
        double startticks = frontLeft.getCurrentPosition();
        setPowerOfAllMotorsTo(-0.5);
        while (1 == 1) {
            double getticks = frontLeft.getCurrentPosition();
            double distance = (getticks - startticks) * (11.867 / 537.6);
            if (-distance >= dist) {
                setPowerOfAllMotorsTo(0);
                break;
            }
        }
    }




}
