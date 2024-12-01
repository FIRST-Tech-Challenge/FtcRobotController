package org.firstinspires.ftc.teamcode.oldCrap;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

//@TeleOp(name = "claw test", group = "test")
public class clawPractice extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        CRServo bottomGrabber = hardwareMap.get(CRServo.class, "bottomGrabber");
        CRServo topGrabber = hardwareMap.get(CRServo.class, "topGrabber");

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        while(opModeIsActive()){
            if(gamepad1.a){
                bottomGrabber.setPower(-1);
                topGrabber.setPower(1);
            }else if(gamepad1.b){
                bottomGrabber.setPower(1);
                topGrabber.setPower(-1);
            }else{
                bottomGrabber.setPower(0);
                topGrabber.setPower(0);
            }
        }
    }
}