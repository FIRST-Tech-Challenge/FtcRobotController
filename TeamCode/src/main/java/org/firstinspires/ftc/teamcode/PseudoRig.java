package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(group = "ZTest")
public class PseudoRig extends LinearOpMode{
    public void runOpMode() throws InterruptedException {
        Servo servoHR = hardwareMap.servo.get("hr");
        DcMotor rr = (DcMotorEx) hardwareMap.dcMotor.get("rr");
        rr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        double targetServo = 0.73;
        int targetPos = 1;//Top height for rig
        if (servoHR.getPosition() < 0.73){
            rr.setTargetPosition(targetPos);
            rr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            servoHR.setPosition(targetServo);
            rr.setTargetPosition(0);
            rr.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        }
        else if(servoHR.getPosition() >= 0.73){
            rr.setTargetPosition(targetPos);//up
            rr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            servoHR.setPosition(0);
            rr.setTargetPosition(0);//down
            rr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
    }
}