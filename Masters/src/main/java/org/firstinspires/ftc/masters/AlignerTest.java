package org.firstinspires.ftc.masters;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="aligner test")
public class AlignerTest extends LinearOpMode {


    CRServo alignerServo;
    Servo claw;

    @Override
    public void runOpMode() throws InterruptedException {

        alignerServo = hardwareMap.get(CRServo.class, "alignerServo");
        claw = hardwareMap.servo.get("clawServo");

        claw.setPosition(BadgerConstants.CLAW_CLOSED);
        waitForStart();
        while(opModeIsActive() && !isStopRequested()){
            if (gamepad1.a){
                alignerServo.setPower(1);
            } else if (gamepad1.b){
                alignerServo.setPower(-1);
            } else {
                alignerServo.setPower(0);
            }
        }
    }
}
