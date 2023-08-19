package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(name="HelloWorldWithMovementPriyanshu",group="Robot")
public class HelloWorldWithMovementPriyanshu extends LinearOpMode {
    public DcMotor leftMotor = null;
    public DcMotor rightMotor = null;

    public DcMotor liftMotor = null;

    public Servo Claw=null;
    @Override
    public void runOpMode() {
        telemetry.addLine("Hello World");
        telemetry.update();
        leftMotor = hardwareMap.get(DcMotor.class, "MotorA");
        rightMotor = hardwareMap.get(DcMotor.class,"MotorB");

        leftMotor.setDirection(DcMotor.Direction.REVERSE);
        rightMotor.setDirection(DcMotor.Direction.FORWARD);
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        Claw=hardwareMap.get(Servo.class,"Servo1");

        liftMotor = hardwareMap.get(DcMotor.class,"MotorC");
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        double leftSpeed;
        double rightSpeed;
        double clawPosition=0;
        double liftSpeed;

        waitForStart();


        while (opModeIsActive()) {
            if (gamepad1.b) {
                telemetry.addData("gamepad1.b", gamepad1.b);
            }

            telemetry.addData("gamepad1.left_stick_y", gamepad1.left_stick_y);
            leftMotor.setPower(gamepad1.left_stick_y);
            rightMotor.setPower(gamepad1.left_stick_y);

            telemetry.addData("gamepad1.right_stick_x", gamepad1.right_stick_x);
            leftMotor.setPower(gamepad1.left_stick_x);
            rightMotor.setPower(gamepad1.left_stick_x);


            if (gamepad1.right_trigger>=0.4) {
                liftMotor.setPower(gamepad1.right_trigger);
                telemetry.addData("liftMotor.getCurrentPosition",liftMotor.getCurrentPosition());
            }
            else if (gamepad1.left_trigger>=0.4) {
                liftMotor.setPower(gamepad1.left_trigger * -1);
                telemetry.addData("liftMotor.getCurrentPosition",liftMotor.getCurrentPosition());
            }
            else {
                liftMotor.setPower(0);
            }
            if (gamepad1.right_bumper&&gamepad1.left_bumper){
                telemetry.addLine("Claw Opened");
                clawPosition=0.3;
                Claw.setPosition(clawPosition);
            }
            else if  (gamepad1.right_bumper||gamepad1.left_bumper){
                //don't need    telemetry.addData("gamepad1.right_bumper",gamepad1.right_bumper);
                telemetry.addLine("claw closed");
                clawPosition=0.7;
                Claw.setPosition(clawPosition);
            }

            if (gamepad1.a){
                liftMotor.setTargetPosition(100);
                liftMotor.setPower(0.5);

                liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            //movement forward and backwards
            if (gamepad1.right_stick_y>0.4){
                rightMotor.setPower(1);
                leftMotor.setPower(1);
            }
            if (gamepad1.right_stick_y<-0.4){
                rightMotor.setPower(-0.6);
                leftMotor.setPower(-0.6);
            }

            telemetry.update();
            sleep(50);

        }
    }
}