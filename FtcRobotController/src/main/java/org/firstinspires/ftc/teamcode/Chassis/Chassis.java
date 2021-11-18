package org.firstinspires.ftc.teamcode.Chassis;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.CRServo;

import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

/**
 * This is the code for mecanum wheels
 * Feel free to use anytime
 */



@TeleOp(name = "MainDriverControl")
public class Chassis extends LinearOpMode{
    //intake servo motors
    private CRServo servo2;
    private CRServo servo1;
    //conveyor belt motor
    private DcMotor ahaha;
    //outtake motors
    private DcMotor leftMotor;
    private DcMotor rightMotor;
    //outtake servo
    private Servo slicey;
    //drivetrain
    /**NOTE DcMotorEx does not work. Heidi fix this later when you arrive.
     private DcMotorEx fl;
     private DcMotorEx fr;
     private DcMotorEx bl;
     private DcMotorEx br;
     **/

    DcMotor fl;
    DcMotor fr;
    DcMotor bl;
    DcMotor br;

    // todo: write your code here

    public void runOpMode(){

        /**NOTE VelocityDrive does not work. Will fix later
         fl = (DcMotorEx)hardwareMap.dcMotor.get(Config.DRIVEFL);
         fr = (DcMotorEx)hardwareMap.dcMotor.get(Config.DRIVEFR);
         bl = (DcMotorEx)hardwareMap.dcMotor.get(Config.DRIVEBL);
         br = (DcMotorEx)hardwareMap.dcMotor.get(Config.DRIVEBR);

         fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
         fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
         bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
         br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

         fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
         fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
         bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
         br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
         //FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
         //FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
         //BL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
         //BR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
         fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
         fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
         bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
         br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

         // Set up the parameters with which we will use our IMU. Note that integration
         // algorithm here just reports accelerations to the logcat log; it doesn't actually
         // provide positional information.
         BNO055IMU.Parameters IMUParameters = new BNO055IMU.Parameters();
         IMUParameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
         IMUParameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
         IMUParameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
         IMUParameters.loggingEnabled      = true;
         IMUParameters.loggingTag          = "IMU";
         IMUParameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

         // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
         // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
         // and named "imu".
         imu = hardwareMap.get(BNO055IMU.class, "imu");
         imu.initialize(IMUParameters);

         // Start the logging of measured acceleration
         imu.startAccelerationIntegration(new Position(), new Velocity(), 100);

         fl.setVelocity((-gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x) * 2500);
         fr.setVelocity((gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x) * 2500);
         bl.setVelocity((-gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x) * 2500);
         br.setVelocity((gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x) * 2500);

         telemetry.addData("FL vel", fl.getVelocity());
         telemetry.addData("FR vel", fr.getVelocity());
         telemetry.addData("BL vel", bl.getVelocity());
         telemetry.addData("BR vel", br.getVelocity());/*
         telemetry.addData("FL pos", fl.getCurrentPosition());
         telemetry.addData("FR pos", fr.getCurrentPosition());
         telemetry.addData("BL pos", bl.getCurrentPosition());
         telemetry.addData("BR pos", br.getCurrentPosition());
         telemetry.addData("imu angle", imu.getAngularOrientation().firstAngle);

         fl = hardwareMap.get(DcMotor.class, "fl");
         fr = hardwareMap.get(DcMotor.class, "fr");
         bl = hardwareMap.get(DcMotor.class, "bl");
         br = hardwareMap.get(DcMotor.class, "br");
         double[] power = new double[4];
         fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
         fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
         fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
         fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
         bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
         bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
         br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
         br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
         **/

        fl = hardwareMap.get(DcMotor.class, "fl");
        fr = hardwareMap.get(DcMotor.class, "fr");
        bl = hardwareMap.get(DcMotor.class, "bl");
        br = hardwareMap.get(DcMotor.class, "br");
        double[] power = new double[4];
        fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        while (opModeIsActive()) {
            power[0]=0;
            power[1]=0;
            power[2]=0;
            power[3]=0;

            //turn left
            if(this.gamepad1.right_stick_x<0){


                power[0]+=this.gamepad1.right_stick_x;
                power[1]+=this.gamepad1.right_stick_x;
                power[2]+=this.gamepad1.right_stick_x;
                power[3]+=this.gamepad1.right_stick_x;

            }

            //turn right
            if(this.gamepad1.right_stick_x>0){

                power[0]+=this.gamepad1.right_stick_x;
                power[1]+=this.gamepad1.right_stick_x;
                power[2]+=this.gamepad1.right_stick_x;
                power[3]+=this.gamepad1.right_stick_x;
            }

            //scooch forward
            if(this.gamepad1.left_stick_y>0){

                power[0]-=this.gamepad1.left_stick_y;
                power[1]+=this.gamepad1.left_stick_y;
                power[2]-=this.gamepad1.left_stick_y;
                power[3]+=this.gamepad1.left_stick_y;

            }

            //scooch backward
            if(this.gamepad1.left_stick_y<0){

                power[0]-=this.gamepad1.left_stick_y;
                power[1]+=this.gamepad1.left_stick_y;
                power[2]-=this.gamepad1.left_stick_y;
                power[3]+=this.gamepad1.left_stick_y;

            }

            //scooch left
            if(this.gamepad1.left_stick_x<0){

                power[0]+=this.gamepad1.left_stick_x;
                power[1]+=this.gamepad1.left_stick_x;
                power[2]-=this.gamepad1.left_stick_x;
                power[3]-=this.gamepad1.left_stick_x;

            }

            //scooch right
            if(this.gamepad1.left_stick_x>0){

                power[0]+=this.gamepad1.left_stick_x;
                power[1]+=this.gamepad1.left_stick_x;
                power[2]-=this.gamepad1.left_stick_x;
                power[3]-=this.gamepad1.left_stick_x;

            }

            //stop-eu
            if(this.gamepad1.b){
                power[0]=0;
                power[1]=0;
                power[2]=0;
                power[3]=0;

            }

            fl.setPower(power[0]);
            fr.setPower(power[1]);
            bl.setPower(power[2]);
            br.setPower(power[3]);


            telemetry.addData("Front left: ", String.valueOf(fl.getPower()));
            telemetry.addData("Front right: ", String.valueOf(fr.getPower()));
            telemetry.addData("Back left: ", String.valueOf(bl.getPower()));
            telemetry.addData("Back right: ", String.valueOf(br.getPower()));



            telemetry.update();


            //intake servo motors
            servo2 = hardwareMap.get(CRServo.class, "servo2");
            servo1 = hardwareMap.get(CRServo.class, "servo1");
            //conveyor belt motor
            ahaha = hardwareMap.get(DcMotor.class, "ahaha");
            //outtake motors
            leftMotor = hardwareMap.get(DcMotor.class, "leftMotor");
            fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightMotor = hardwareMap.get(DcMotor.class, "rightMotor");
            //outtake servo
            slicey = hardwareMap.get(Servo.class, "slicey");

            telemetry.addData("Status", "Press Play!");
            telemetry.update();

            boolean powered = false;
            boolean shooting = false;
            boolean intaking = false;
            int outtakeCounter = 0;
            int conveyorCounter = 0;
            int sliceyCounter = 0;
            int intakeCounter = 0;

            if(gamepad1.x && conveyorCounter == 0){
                conveyorCounter = 200;
                powered = !powered;

            }
            if(conveyorCounter > 0){
                conveyorCounter = conveyorCounter-1;
            }
            telemetry.addData("Conveyor status", conveyorCounter);
            telemetry.update();

            if (powered) {
                ahaha.setPower(-.6);

            }
            else{
                ahaha.setPower(0.0);

            }


            if(gamepad1.right_trigger>0 && outtakeCounter == 0){
                outtakeCounter = 200;
                shooting = !shooting;

            }
            if(outtakeCounter > 0){
                outtakeCounter = outtakeCounter-1;
            }
            telemetry.addData("Outtake status", outtakeCounter);
            telemetry.update();

            if (shooting) {

                leftMotor.setTargetPosition(10000);
                rightMotor.setTargetPosition(10000);
                leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                leftMotor.setPower(0.8);
                rightMotor.setPower(0.8);
            }
            else{
                leftMotor.setTargetPosition(0);
                leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightMotor.setTargetPosition(0);
                rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                leftMotor.setPower(0.0);
                rightMotor.setPower(0.0);

            }


            if (this.gamepad1.left_trigger > 0 && sliceyCounter == 0){
                slicey.setPosition(0.9);
                sliceyCounter = 100;

            }
            else if (sliceyCounter == 0){

                slicey.setPosition(-0.9);
            }
            if(sliceyCounter > 0){
                sliceyCounter = sliceyCounter-1;
            }
            telemetry.addData("Slicey status", sliceyCounter);
            telemetry.update();




            if(gamepad1.a && intakeCounter == 0){
                intakeCounter = 200;
                intaking = !intaking;

            }
            if(intakeCounter > 0){
                intakeCounter = intakeCounter-1;
            }
            telemetry.addData("Intake status", intakeCounter);
            telemetry.update();

            if (intaking) {

                ahaha.setPower(0.6);
                servo2.setPower(-0.9);
                servo1.setPower(0.9);

            }
            else{

                ahaha.setPower(-0.6);
                servo2.setPower(0.9);
                servo1.setPower(-0.9);

            }
        }

    }

}




