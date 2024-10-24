package org.firstinspires.ftc.teamcode.Opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp
public class Reset extends LinearOpMode {
    private DcMotorEx leftFront, leftBack, rightFront, rightBack, arm, slides;
    public void runOpMode(){
        //80.5 180 -624
        leftFront = hardwareMap.get(DcMotorEx.class, "bl");
        leftBack = hardwareMap.get(DcMotorEx.class, "fl");
        rightFront = hardwareMap.get(DcMotorEx.class, "fr");
        rightBack = hardwareMap.get(DcMotorEx.class, "br");
        arm = hardwareMap.get(DcMotorEx.class, "arm");
        slides = hardwareMap.get(DcMotorEx.class, "slides");

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        waitForStart();
    }
}
