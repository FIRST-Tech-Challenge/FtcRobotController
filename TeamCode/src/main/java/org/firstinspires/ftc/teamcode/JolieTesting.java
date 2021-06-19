package org.firstinspires.ftc.teamcode;

/*
* Jolie Testing File
* 19 June 2021
* */


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;


//Specify the opMode Type
@TeleOp(name = "Jolie Testing")
public class JolieTesting extends LinearOpMode {//needs which type
    //Motors
    private DcMotor motorFR, motorFL, motorBL, motorBR;//drive train
    private DcMotor outtake;
    private DcMotor arm;

    //servos
    private CRServo leftIntake, rightIntake;
    private Servo claw;

    //sensor
    private UltrasonicSensor usSensor;

    @Override
    public void runOpMode() throws InterruptedException{
        motorFR = hardwareMap.dcMotor.get("FR");
        motorFL = hardwareMap.dcMotor.get("FL");
        motorBL = hardwareMap.dcMotor.get("BL");
        motorBR = hardwareMap.dcMotor.get("BR");
        outtake = hardwareMap.dcMotor.get("outtake");
        arm = hardwareMap.dcMotor.get("arm");

        leftIntake = hardwareMap.crservo.get("leftIntake");
        rightIntake = hardwareMap.crservo.get("rightIntake");

        claw = hardwareMap.servo.get("claw");

        usSensor = hardwareMap.ultrasonicSensor.get("usSensor");

        //other needed variables: to change speeds of the motors
        double powerMod = 1.0;
        double intakeMod = 1.0;
        double outtakeMod = 0.64;
        double armMod = 0.3;

        waitForStart();

        while(opModeIsActive()){
            //gamepad1 is for driving
            //gamepad2 is for attachments

            //everything driving
            if(gamepad1.right_bumper){//boolean
                powerMod = 0.5;
            }else{
                powerMod = 1.0;
            }

            //Mecanum drive using trig
            double angle = Math.atan2(gamepad1.right_stick_y, gamepad1.right_stick_x) - (Math.PI/4);
            double r = Math.hypot(gamepad1.right_stick_x, gamepad1.right_stick_y);
            double rotation = gamepad1.left_stick_x;

            double powerOne = r*Math.sin(angle);
            double powerTwo = r*Math.cos(angle);

            motorFL.setPower((powerOne - (rotation))*powerMod);
            motorFR.setPower((powerTwo + (rotation))*powerMod);
            motorBL.setPower((powerTwo - (rotation))*powerMod);
            motorBR.setPower((powerOne + (rotation))*powerMod);



            //everything intake
            double intakeSpeed = gamepad1.left_trigger * intakeMod;//trigger returns a range of values
            leftIntake.setPower(intakeSpeed);
            rightIntake.setPower(intakeSpeed);//crservos == dcmotors


            if(gamepad1.a){//reverse direction
                intakeMod = -1.0;
            }
            else{
                intakeMod = 1.0;
            }


            if(gamepad2.x){
                claw.setPosition(1);//open
            }
            if(gamepad2.y){
                claw.setPosition(0);//close
            }

            if(usSensor.getUltrasonicLevel() > 0.6){
                leftIntake.setPower(1);
                rightIntake.setPower(1);
                Thread.sleep(1000);
                leftIntake.setPower(0);
                rightIntake.setPower(0);
            }




        }
    }


}
