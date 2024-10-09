package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "TeleOp Test V1")
public class TeleOp_testV1 extends LinearOpMode {

    Servo Thing1;

    Servo Elbow;

    Servo Thing2;

    DcMotor sideSlide;

    DcMotor upSlide;
    double tgt = 0;
    double tgtslide;

    //Slides Pos Variables
    double sideSlidePos;
    double upSlidePos;

    @Override
    public void runOpMode() throws InterruptedException {

        //HardwareMap

        Thing1 = hardwareMap.get(Servo.class, "Thing1");

        Elbow = hardwareMap.get(Servo.class, "Elbow");

        Thing2 = hardwareMap.get(Servo.class, "Thing2");

        sideSlide = hardwareMap.get(DcMotor.class, "sideSlide");

        upSlide = hardwareMap.get(DcMotor.class, "upSlide");

        //Declare motor position variables

        sideSlidePos = sideSlide.getCurrentPosition();
        upSlidePos = upSlide.getCurrentPosition();


        // Motor Configuration

        Motor_Config();

        waitForStart();

        sideSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        sideSlide.setPower(1);
        upSlide.setPower(1);
        Thing2.setPosition(0);
        sideSlide.setTargetPosition(0);
        upSlide.setTargetPosition(150);

        while(opModeIsActive())
        {

            sideSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            upSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            while (sideSlide.isBusy() || upSlide.isBusy() && !isStopRequested())
            {
                idle();
            }


            //Thing1 and Elbow Button Control

            if(gamepad1.a)
            {
                Thing1.setPosition(0);
            }
            else if(gamepad1.y){
                Thing1.setPosition(1);
            }
            else{
                Thing1.setPosition(0.5);
            }
            if(gamepad1.x){
                Elbow.setPosition(0.9);

            }
            else if(gamepad1.b)
            {
                Elbow.setPosition(0.24);
            }


            telemetry.addData("Elbow", Elbow.getPosition());
            telemetry.addData("Thing1", Thing1.getPosition());
            telemetry.addData("Thing2", Thing2.getPosition());
            telemetry.addData("SideSlide", sideSlidePos);
            telemetry.addData("UpSlide", upSlidePos);
            telemetry.update();

            //Servo Test code
//            tgt = -this.gamepad1.left_stick_y;
//            Thing2.setPosition(tgt);

            if(gamepad1.left_bumper)
            {
                sideSlidePos += 50;

            }
            else if(gamepad1.right_bumper)
            {
                sideSlidePos -= 50;
            }

            if(gamepad1.dpad_right)
            {
                Thing2.setPosition(0);
            }
            else if(gamepad1.dpad_left)
            {
                Thing2.setPosition(1);
            }


            sideSlide.setTargetPosition((int) sideSlidePos);

            sideSlidePos = Math.min(Math.max(sideSlidePos, 0), 3000);
            upSlidePos = Math.min(Math.max(upSlidePos, 0), 3000);

            if(gamepad1.dpad_up)
            {
                upSlide.setTargetPosition(2000);
            }
            else if(gamepad1.dpad_down)
            {
                upSlide.setTargetPosition(0);
            }

            sideSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            upSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        }


    }

    public void Motor_Config()
    {
        sideSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        upSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        sideSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        sideSlide.setDirection(DcMotorSimple.Direction.REVERSE);
        upSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        upSlide.setDirection(DcMotorSimple.Direction.FORWARD);
    }


}
