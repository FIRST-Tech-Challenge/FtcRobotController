package org.firstinspires.ftc.teamcode;
//Test
//By Ethan Clawsie and Aman Sulaiman, 2021-2022 Freight Frenzy

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.TouchSensor;


public class OutreachHardware {
    public DcMotor frontLeft, backLeft, frontRight, backRight, cascadeMotorRight, cascadeMotorLeft, arm;
    public TouchSensor touchRight, touchLeft;
    public ColorSensor colorSensor;
    public Servo claw;
    public CRServo wrist;
    public DistanceSensor distanceSensor;
    HardwareMap hwMap;
    static final double COUNTS_PER_MOTOR_REV = 537.7;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // No External Gearing.
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double DRIVE_SPEED = 0.6;
    static final double TURN_SPEED = 0.5;
    public ElapsedTime timer = new ElapsedTime();

    public void initTeleOpIMU(HardwareMap hwMap) {
        this.hwMap = hwMap;
        timer.reset();
        backLeft = hwMap.dcMotor.get("lb");
        backRight = hwMap.dcMotor.get("rb");
        frontLeft = hwMap.dcMotor.get("lf");
        frontRight = hwMap.dcMotor.get("rf");
        //colorSensor  = hwMap.colorSensor.get("colorSensor");
        //distanceSensor = (DistanceSensor)hwMap.get("distanceSensor");
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        claw = hwMap.servo.get("claw");
//        arm = hwMap.dcMotor.get("arm");
//        wrist = hwMap.crservo.get("wrist");
//        //turntable = hwMap.dcMotor.get("turntable");
        //cascadeMotorRight = hwMap.dcMotor.get("cascadeMotorRight");
        //cascadeMotorLeft = hwMap.dcMotor.get("cascadeMotorLeft");
//        //cascadeTouch = hwMap.touchSensor.get("cascade_touch");
////        bucket = hwMap.servo.get("bucket");
        //intake1 = hwMap.crservo.get("intakeServoRight");
        //intake2 = hwMap.crservo.get("intakeServoLeft");
        //slider = hwMap.dcMotor.get("slider");
        //touchRight = hwMap.touchSensor.get("touch_right");
        //touchLeft = hwMap.touchSensor.get("touch_left");

        boolean go = true;

        //bucket.setPosition(.9);
    }


    public void setPowerOfAllMotorsToForTime(double power, double time) {
        timer.reset();
        while (timer.seconds() <= time) {
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
    public void strafeRightForTime(double power, double time) {
        timer.reset();
        while (timer.seconds() <= time) {
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
    public void strafeLeftForTime(double power, double time) {
        timer.reset();
        while (timer.seconds() <= time) {
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
    public void turnRightForTime(double power, double time) {
        timer.reset();
        while (timer.seconds() <= time) {
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
    public void turnLeftForTime(double power, double time) {
        timer.reset();
        while (timer.seconds() <= time) {
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
    public double getTime() {
        return timer.time();
    }

    public void setPowerOfAllMotorsTo(double power) {
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
    public void cascadeupfortime(double power, double time) {
        timer.reset();
        while (timer.seconds() <= time) {
            cascadeMotorRight.setPower(power);
            cascadeMotorLeft.setPower(power);
        }
        cascadeMotorRight.setPower(0.1);
        cascadeMotorLeft.setPower(0.1);
    }

    /*public void intakeForTime(double power, double time) {
        timer.reset();
        while (timer.seconds() <= time) {
            intake1.setPower(power);
            intake2.setPower(-power);
        }

    }*/
    /*public void carouselpowerfortime(double power, double time) {
        timer.reset();
        while (timer.seconds() <= time) {
            carousel.setPower(power);
        }
        carousel.setPower(0);
    }*/
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
    public int inchesToTicksCascadeOld(int inches){
        int cpi = (int) (384.5/ (2* 3.14159265));
        return inches * cpi;
    }
    public int inchesToTicksOldDrive(double inches){
        int cpi = (int) (537.7 / (3.78* 3.14159265));
        return (int) (inches * cpi);
    }
    public int degreesToTicks(int degrees){
        int ticksPerDegree = (int)(2786.2/360);
        return ticksPerDegree * degrees;
    }

}
//650, -17, -650, -2, 1277, 609, -14,
// 653, -8, -652, 647, 1280,
// 646, -3,