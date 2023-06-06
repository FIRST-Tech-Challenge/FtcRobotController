package org.firstinspires.ftc.masters;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="aligner test")
public class AlignerTest extends LinearOpMode {


    Servo alignerServo;
    Servo claw;

    @Override
    public void runOpMode() throws InterruptedException {

        alignerServo = hardwareMap.get(Servo.class, "alignerServo");
        claw = hardwareMap.servo.get("clawServo");

        claw.setPosition(BadgerConstants.CLAW_CLOSED);
        waitForStart();
        while(opModeIsActive() && !isStopRequested()){
            if (gamepad1.a){
                alignerServo.setPosition(1);
            } else if (gamepad1.b) {
                alignerServo.setPosition(-1);
            }
        }
    }
}
