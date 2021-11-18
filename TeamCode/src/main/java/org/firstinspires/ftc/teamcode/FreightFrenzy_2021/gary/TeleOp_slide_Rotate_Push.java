package org.firstinspires.ftc.teamcode.FreightFrenzy_2021.gary;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;

import java.util.ArrayList;
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



        waitForStart();

        while(opModeIsActive()){

            //Slide
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

            //Push
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

            telemetry.addLine("Slide Current: " + Slide.getCurrentPosition());
            telemetry.update();


        }

    }
}