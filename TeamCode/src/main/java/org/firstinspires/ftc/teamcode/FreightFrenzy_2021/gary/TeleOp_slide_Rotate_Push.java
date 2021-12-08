package org.firstinspires.ftc.teamcode.FreightFrenzy_2021.gary;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "TeleOp_slide_Rotate_Push", group = "Linear OpMode")
public class TeleOp_slide_Rotate_Push extends LinearOpMode{
    private DcMotor Slide = null;
    private Servo Rotate = null;
    private Servo Push = null;



    @Override
    public void runOpMode(){
        Slide = hardwareMap.get(DcMotor.class , "Slide");
        Rotate = hardwareMap.get(Servo.class , "Rotate");
        Push = hardwareMap.get(Servo.class , "Push");


        Slide.setDirection(DcMotor.Direction.FORWARD);
        Rotate.setDirection(Servo.Direction.FORWARD);

        int Fposition = Slide.getCurrentPosition();
        int Sposition = Fposition + 700;
        int Tposition = Fposition + 1400;
        Push.setPosition(0);
        Rotate.setPosition(0.03);

        boolean Fposition1 = true;
        boolean Sposition1 = true;
        boolean Tposition1 = true;
        boolean Push1 = true;
        boolean Rotate1 = true;




        waitForStart();

        while(opModeIsActive()){

            if(gamepad1.a){
                if(Fposition1) {
                    Slide.setTargetPosition(Fposition);
                    Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Slide.setPower(0.5);
                }
                Fposition1 = false;
            }else if(!Fposition1){
                Fposition1 = true;
            }

            if(gamepad1.b){
                if(Sposition1) {
                    Slide.setTargetPosition(Sposition);
                    Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Slide.setPower(0.5);
                }
                Sposition1 = false;
            }else if(!Sposition1){
                Sposition1 = true;
            }

            if(gamepad1.y){
                if(Tposition1) {
                    Slide.setTargetPosition(Tposition);
                    Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Slide.setPower(0.5);
                }
                Tposition1 = false;
            }else if(!Tposition1){
                Tposition1 = true;
            }


            if(gamepad2.left_trigger == 1){
                if(Rotate1 && Rotate.getPosition() > 0.5) {
                    Rotate.setPosition(0.03);
                    telemetry.addLine("Inital");
                }else if(Rotate1 && Rotate.getPosition() < 0.5){
                    Rotate.setPosition(1);
                    telemetry.addLine("Face outside");
                }

                Rotate1 = false;
            }else if(!Rotate1){
                Rotate1 = true;
            }

            if(gamepad2.right_trigger == 1){
                if (Push1){
                    Push.setPosition(0);
                    Push1 = false;
                }
            } else if (!Push1){
                Push.setPosition(0.4);
                Push1 = true;
            }



            telemetry.addLine("Slide Current: " + Slide.getCurrentPosition());
            telemetry.update();


        }

    }
}
/*

            if(gamepad1.a == true){
                Slide.setTargetPosition(Fposition);
                Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Slide.setPower(0.5);
            }else if(gamepad1.x == true){
                Slide.setTargetPosition(Sposition);
                Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Slide.setPower(0.5);
            }else if(gamepad1.y == true){
                Slide.setTargetPosition(Tposition);
                Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Slide.setPower(0.5);
            }


            if(gamepad1.left_bumper == true){
                Push.setPosition(0.4);
            }else if(gamepad1.right_bumper == true && Push.getPosition() > 0.2){
                Push.setPosition(0);
            }

            if(gamepad1.right_trigger == 1){
                Rotate.setPosition(1);
            } else if(gamepad1.left_trigger ==1){
                Rotate.setPosition(0.03);
            }
 */