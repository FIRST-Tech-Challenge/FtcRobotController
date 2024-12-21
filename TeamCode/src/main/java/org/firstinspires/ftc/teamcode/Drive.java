package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.auto.BasicRobot;


@TeleOp
public class Drive extends LinearOpMode implements BasicRobot {

    //motors for drive
    private DcMotor frontrightDrive = null;
    private DcMotor backrightDrive = null;
    private DcMotor frontleftDrive = null;
    private DcMotor backleftDrive = null;

    // variables for movement of robot;
    private float y ;
    private float x ;
    private float rx;

    private boolean claws = false;
    private boolean bpressed = true;
    private boolean apressed = true;
    private boolean ypressed = true;
    private boolean gpdpressed = true;
    private boolean abort = false;

    private final double maxPower = 0.6;

    public void drive(){//DcMotor backleftDrive, DcMotor backrightDrive, DcMotor frontleftDrive, DcMotor frontrightDrive) {
        if((rx<0)){
            rx=-0.65F;
        }if((rx>0)){
            rx=0.65F;
        }
        backleftDrive.setPower(RangeLimit(y+(x)-rx)); //backR
        backrightDrive.setPower(RangeLimit(y-(x)+rx)); //frontL
        frontleftDrive.setPower(RangeLimit(y-(x)-rx));  //frontR
        frontrightDrive.setPower(RangeLimit(y+(x)+rx));
        telemetry.addData("drive", RangeLimit(y+x+rx));

    }

    private double RangeLimit(double value){
        double denominator = 0;

        if(Math.abs(y) > 0.2){
            denominator++;
        }
        if(Math.abs(x) > 0.2){
            denominator++;
        }
        if(!(rx==0)){
            denominator++;
        }
        telemetry.addData(" dem", denominator);
        return (value /  denominator) * maxPower;

    }



    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");

        ElapsedTime runtime = new ElapsedTime();
        //define robots attachments to work
        frontrightDrive = hardwareMap.get(DcMotor.class, "frontright");
        backrightDrive = hardwareMap.get(DcMotor.class, "backright");
        frontleftDrive = hardwareMap.get(DcMotor.class, "frontleft");
        backleftDrive = hardwareMap.get(DcMotor.class, "backleft");
        //load other motors
        DcMotor elavator1 = hardwareMap.get(DcMotor.class, "elavator1");
        DcMotor elavator2 = hardwareMap.get(DcMotor.class, "elavator2");

        //load servos
        outtakeAngle.setServo(hardwareMap.get(Servo.class, outtakeAngle.servoName));
        intakeAngle.setServo(hardwareMap.get(Servo.class, intakeAngle.servoName));
        intakeSlide1.setServo(hardwareMap.get(Servo.class, intakeSlide1.servoName));
        intakeSlide2.setServo(hardwareMap.get(Servo.class, intakeSlide2.servoName));
        outtakeClaw.setServo(hardwareMap.get(Servo.class, outtakeClaw.servoName));
        intakeClaw.setServo(hardwareMap.get(Servo.class, intakeClaw.servoName));
//        Servo outtakeClaw = hardwareMap.get(Servo.class, "outtakeClaw");
//        Servo intakeClaw = hardwareMap.get(Servo.class, "intakeClaw");


        //reverse correct motor so power of 1 makes robot go forward
        frontrightDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        //set start position for teleOp
        intakeSlide1.startServo();
        intakeSlide2.startServo();
        intakeAngle.set(TRANSFER);
        outtakeAngle.set(TRANSFER);
        intakeClaw.set(OPEN);//intakeClawPosition.open.position);
        outtakeClaw.set(CLOSE);//outtakeClawPosition.close.position);
        claws=false;


        //imu setup
        IMU imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT));
        imu.initialize(parameters);

        //send telemetry data and wait for start
        telemetry.update();
        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            //is used a constant, so it stay at its current position
            double elavatorPower = 0.1;

            //define the controller positions
            y = gamepad1.left_stick_y;
            x = gamepad1.left_stick_x;
            rx = gamepad1.right_stick_x;
            if(gamepad1.dpad_down || gamepad1.dpad_up || gamepad1.dpad_right || gamepad2.dpad_left){
                y = 0;
                x = 0;
                rx = 0;
                if (gamepad1.dpad_up){
                    y= -0.35F;
                }
                if (gamepad1.dpad_down){
                    y= 0.35F;
                }
                if (gamepad1.dpad_left){
                    x= -0.35F;
                }
                if (gamepad1.dpad_right){
                    x= 0.35F;
                }
                drive();
            } else {
                drive();
            }
            telemetry.addData("Servo Position 2", (double) intakeSlide2.getPos());
            telemetry.addData("Servo Position 1", (double) intakeSlide1.getPos());
            telemetry.addData("preref", gamepad1.dpad_left);
            //pressed
            telemetry.addData("bpressed", bpressed);
            telemetry.addData("apressed", apressed);
            telemetry.addData("ypressed", ypressed);

            if (gamepad2.a && apressed == true) {
                if (!claws) {
                    intakeClaw.set(CLOSE);//intakeClawPosition.close.position);
                    waitMe(0.4, runtime);
                    outtakeClaw.set(OPEN);
//                    outtakeClaw.setPosition(outtakeClawPosition.open.position);
                    claws = true;
                } else if (claws) {
                    outtakeClaw.set(CLOSE);
//                    outtakeClaw.setPosition(outtakeClawPosition.close.position);
                    waitMe(0.4, runtime);
                    intakeClaw.set(OPEN);
//                    intakeClaw.setPosition(intakeClawPosition.open.position);
                    claws = false;
                }
                apressed = false;
            }
            if (gamepad2.b && bpressed == true) {
                if (intakeAngle.get(GRAB) + 0.1 > intakeAngle.getPos() && intakeAngle.get(GRAB) - 0.1 < intakeAngle.getPos()) {
                    intakeAngle.set(TRANSFER);
                } else if (intakeAngle.get(TRANSFER) + 0.1 > intakeAngle.getPos() && intakeAngle.get(TRANSFER) - 0.1 < intakeAngle.getPos()) {
                    intakeAngle.set(GRAB);
                }
                bpressed = false;
            }
            if (gamepad2.y && ypressed == true) {
                if (outtakeAngle.get(PLACE) + 0.05 > outtakeAngle.getPos() && outtakeAngle.get(PLACE) - 0.05 < outtakeAngle.getPos()) {
                    outtakeAngle.set(0);
                } else if (outtakeAngle.get(TRANSFER) + 0.05 > outtakeAngle.getPos() && outtakeAngle.get(TRANSFER) - 0.05 < outtakeAngle.getPos()) {
                    outtakeAngle.set(1);
                }
                ypressed = false;
            }
            if (gamepad2.dpad_down && gpdpressed == true) {
                outtakeClaw.set(OPEN);//outtakeClawPosition.open.position);
                intakeClaw.set(CLOSE);//intakeClawPosition.close.position);
                claws=true;
                waitMe(0.3, runtime);
                if(abort){break;}
                intakeAngle.set(TRANSFER);
                waitMe(0.3, runtime);
                if(abort){break;}
                intakeSlide1.set(0.0);
                intakeSlide2.set(1.0);
                waitMe(0.6, runtime);
                if(abort){break;}
                outtakeClaw.set(CLOSE);//outtakeClawPosition.close.position);
                waitMe(0.3, runtime);
                if(abort){break;}
                intakeClaw.set(OPEN);//intakeClawPosition.open.position);
                claws=false;

            }
            if (gamepad2.right_trigger > 0.2) {
//                intakeSlide1.setPosition(0.35/*-0.15*/);
//                intakeSlide2.setPosition(0.65/*+0.15*/);

                intakeSlide1.decrease();
                intakeSlide2.increase();
            }

            if (gamepad2.left_trigger > 0.2) {
                intakeSlide1.increase();
                intakeSlide2.decrease();
            }

            if (gamepad2.left_stick_y < -0.2) {
                elavatorPower = 0.9;
            }
            if (gamepad2.left_stick_y > 0.2) {
                elavatorPower = -0.7;
            }

            //if not pressed button change
            if (!gamepad2.b) {
                bpressed = true;
            }
            if (!gamepad2.y) {
                ypressed = true;
            }
            if (!gamepad2.a) {
                apressed = true;
            }
            if(!gamepad2.dpad_down){
                gpdpressed = true;
            }
            if(gamepad2.x) {
                abort = true;
            }

            //set motor power
            elavator2.setPower(elavatorPower);
            elavator1.setPower(-elavatorPower);
            telemetry.update();
        }
    }
    private void waitMe(double sec, ElapsedTime runtime){
        runtime.reset();
        while (runtime.seconds() < sec && abort==false) {
        }
    }
}

