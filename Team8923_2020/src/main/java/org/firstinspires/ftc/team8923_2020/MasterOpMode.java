package org.firstinspires.ftc.team8923_2020;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public abstract class MasterOpMode extends LinearOpMode {

        DcMotor motorFR;
        DcMotor motorFL;
        DcMotor motorBR;
        DcMotor motorBL;

        //DcMotor intakeLeft;
        //DcMotor intakeRight;

        //DcMotor motorLift1;
        //DcMotor motorLift2;


        BNO055IMU imu;

        //Servo wobbleGrabber;



        public void initHardware(){

            motorFR = hardwareMap.dcMotor.get("motorFR");
            motorFL = hardwareMap.dcMotor.get("motorFL");
            motorBR = hardwareMap.dcMotor.get("motorBR");
            motorBL = hardwareMap.dcMotor.get("motorBL");

            //intakeLeft = hardwareMap.dcMotor.get("intakeLeft");
            //intakeRight = hardwareMap.dcMotor.get("intakeRight");

            //motorLift1 = hardwareMap.dcMotor.get("motorLift1");
            //motorLift2 = hardwareMap.dcMotor.get("motorLift2");

            //wobbleGrabber = hardwareMap.servo.get("wobbleGrabber");

            motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //motorLift1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            //motorLift2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            motorFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motorFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motorBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motorBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            //intakeLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            //intakeRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            //motorLift1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            //motorLift2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
            parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
            imu = hardwareMap.get(BNO055IMU.class, "imu");
            imu.initialize(parameters);

            while (!isStopRequested() && !imu.isGyroCalibrated()){
                sleep(50);
                idle();
            }
        }

        public void driveMecanum(double driveAngle, double turnPower, double drivePower){
            double x = drivePower * Math.cos(driveAngle);
            double y = drivePower * Math.sin(driveAngle);

            double powerFL = x + y - turnPower;
            double powerFR = y - x + turnPower;
            double powerBL = y - x - turnPower;
            double powerBR = y + x + turnPower;

            motorFR.setPower(powerFR);
            motorFL.setPower(powerFL);
            motorBR.setPower(powerBR);
            motorBL.setPower(powerBL);


        }

        //Used for calculating distance between 2 points
        public double calculateDistance(double deltaX, double deltaY){
            return Math.sqrt(Math.pow(deltaX, 2) + Math.pow(deltaY, 2));
        }




    }


