package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


@TeleOp
public class Drive extends LinearOpMode {

    private boolean claws = false;
    private boolean bpressed = true;
    private boolean apressed = true;

    private boolean ypressed = true;
    private int iop = 0;

    private double TRANSFERINTAKE = 0.73;
    private double RETRIEV = 0.08;
    private double MOTION = 0.5;
    //"PLACE" EFFECTS THE FLIP OUT, "TRANSFEROUTTAKE" EFFECTS THE TRANSFER POSITION
    private double PLACE = 0.58;
    private double UP = 0.5;
    private double TRANSFEROUTTAKE = 0.44;


    private enum intakeClawPosition{

        open(0.0),
        close(0.117);

        private double position;

        intakeClawPosition(double v) {
            this.position = v;
        }
    }

    private enum outtakeClawPosition{

        open(0.036),
        close(0.28);

        private double position;

        outtakeClawPosition(double v) {
            this.position = v;
        }
    }



    private final double DEADZONE = .1;
    private final double speedDivider = 2;
    private final double maxPower = 0.72;


    public void drive(DcMotor backleftDrive, DcMotor backrightDrive, DcMotor frontleftDrive, DcMotor frontrightDrive) {
        backleftDrive.setPower(backLeftPower()); //backR
        backrightDrive.setPower(backRightPower()); //frontL
        frontleftDrive.setPower(frontLeftPower());  //frontR
        frontrightDrive.setPower(frontRightPower());
        telemetry.addData("drive", gamepad1.left_stick_y);

    }
    private double backLeftPower(){
        double power=RangeLimit(gamepad1.left_stick_y+gamepad1.left_stick_x-gamepad1.right_stick_x);
        telemetry.addData("backLeftPower", power);
        return power;
    }
    private double backRightPower(){
        double power=RangeLimit(gamepad1.left_stick_y-gamepad1.left_stick_x+gamepad1.right_stick_x);
        telemetry.addData("backRightPower", power);
        return power;
    }
    private double frontLeftPower(){
        double power=RangeLimit(gamepad1.left_stick_y-gamepad1.left_stick_x-gamepad1.right_stick_x);
        telemetry.addData("frontLeftPower", power);
        return power;
    }
    private double frontRightPower(){
        double power=RangeLimit(gamepad1.left_stick_y+gamepad1.left_stick_x+gamepad1.right_stick_x);
        telemetry.addData("frontRightPower", power);
        return power;
    }

    private double RangeLimit(double value){
        double denominator = Math.max(Math.abs(gamepad1.left_stick_y) + Math.abs(gamepad1.left_stick_x) + Math.abs(gamepad1.right_stick_x), 1);
        return (value /  denominator) * maxPower;

    }



    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");

        ElapsedTime runtime = new ElapsedTime();
        //define robots attachments to work
        DcMotor frontrightDrive = hardwareMap.get(DcMotor.class, "frontright");
        DcMotor backrightDrive = hardwareMap.get(DcMotor.class, "backright");
        DcMotor frontleftDrive = hardwareMap.get(DcMotor.class, "frontleft");
        DcMotor backleftDrive = hardwareMap.get(DcMotor.class, "backleft");

        Servo outtakeAngle = hardwareMap.get(Servo.class, "outtakeAngle");
        Servo outtakeClaw = hardwareMap.get(Servo.class, "outtakeClaw");

        Servo intakeAngle = hardwareMap.get(Servo.class, "intakeAngle");
        Servo intakeClaw = hardwareMap.get(Servo.class, "intakeClaw");

        Servo intakeSlide1 = hardwareMap.get(Servo.class, "intakeSlide1");
        Servo intakeSlide2 = hardwareMap.get(Servo.class, "intakeSlide2");

        DcMotor elavator1 = hardwareMap.get(DcMotor.class, "elavator1");
        DcMotor elavator2 = hardwareMap.get(DcMotor.class, "elavator2");

        //reverse correct motor so power of 1 makes robot go forward
        frontrightDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        //set start position for teleOp
        intakeAngle.setPosition(MOTION);
        outtakeAngle.setPosition(TRANSFEROUTTAKE);
        intakeClaw.setPosition(intakeClawPosition.open.position);
        outtakeClaw.setPosition(outtakeClawPosition.close.position);

        IMU imu = hardwareMap.get(IMU.class,"imu");

        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT));
        imu.initialize(parameters);

        Orientation start = imu.getRobotOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

        //send telemetry data and wait for start
        telemetry.update();
        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            double elavatorPower = 0.1;
            drive(backleftDrive, backrightDrive, frontleftDrive, frontrightDrive);
            telemetry.addData("left_stick_x", gamepad1.left_stick_x);
            telemetry.addData("left_stick_y", gamepad1.left_stick_y);
            telemetry.addData("Servo Position", (double)intakeAngle.getPosition());
            telemetry.addData("Servo Position out", (double)outtakeAngle.getPosition());

            telemetry.addData("bpressed", bpressed);
            telemetry.addData("ypressed", ypressed);
            telemetry.addData("iop", iop);

            if(gamepad2.a && apressed ==true){
                if(!claws){
                    intakeClaw.setPosition(intakeClawPosition.close.position);
                    wait(0.3, runtime);
                    outtakeClaw.setPosition(outtakeClawPosition.open.position);
                    claws = true;
                } else if(claws) {
                    outtakeClaw.setPosition(outtakeClawPosition.close.position);
                    wait(0.3, runtime);
                    intakeClaw.setPosition(intakeClawPosition.open.position);

                    claws = false;
                }
                apressed =false;
            }


            if(gamepad2.right_trigger > 0.2){
                intakeSlide1.setPosition(0.35/*-0.15*/);
                intakeSlide2.setPosition(0.65/*+0.15*/);
            }
            if(gamepad2.left_trigger > 0.2){
                intakeSlide1.setPosition(0);
                intakeSlide2.setPosition(1);

            }
            if(gamepad2.b && bpressed ==true){
//                if (MOTION+0.1>intakeAngle.getPosition() && MOTION-0.1<intakeAngle.getPosition()){
                    intakeAngle.setPosition(RETRIEV);
//                }
                if (RETRIEV+0.1>intakeAngle.getPosition()&& RETRIEV-0.1<intakeAngle.getPosition()){
                    intakeAngle.setPosition(TRANSFERINTAKE);
                }
                else if (TRANSFERINTAKE+0.1>intakeAngle.getPosition() && TRANSFERINTAKE-0.1<intakeAngle.getPosition()){
                    intakeAngle.setPosition(RETRIEV);
                }
                bpressed =false;
            }
            if(gamepad2.y && ypressed==true){
                if (PLACE +0.05>outtakeAngle.getPosition() && PLACE -0.05<outtakeAngle.getPosition()){
                    outtakeAngle.setPosition(TRANSFEROUTTAKE);
                    iop++;
                }
                else if (TRANSFEROUTTAKE +0.05>outtakeAngle.getPosition()&& TRANSFEROUTTAKE -0.05<outtakeAngle.getPosition()){
                    outtakeAngle.setPosition(PLACE);
                    iop++;
                }
                ypressed=false;
            }

            if(gamepad2.left_stick_y < -0.2){
                elavatorPower = 0.9;
            }
            if(gamepad2.left_stick_y > 0.2){
                elavatorPower = -0.7;
            }


            if(!gamepad2.b){
                bpressed =true;
            }
            if(!gamepad2.y){
                ypressed=true;
            }
            if(!gamepad2.a){
                apressed =true;
            }
            elavator2.setPower(elavatorPower);
            elavator1.setPower(-elavatorPower);
            telemetry.addData("x", start.firstAngle - imu.getRobotOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).firstAngle);
            telemetry.addData("y", start.secondAngle - imu.getRobotOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).secondAngle);
            telemetry.addData("z", start.thirdAngle - imu.getRobotOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle);
            telemetry.update();

        }
    }
    private void wait(double sec, ElapsedTime runtime){
        runtime.reset();
        while(runtime.seconds() < sec){

        }
    }
}
