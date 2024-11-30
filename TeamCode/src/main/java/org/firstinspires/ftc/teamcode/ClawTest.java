package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name = "Claw Test")
public class ClawTest extends LinearOpMode {

    private boolean OuttakeClawClosed = false;
    private ElapsedTime ClawTime = new ElapsedTime();

    @Override
    public void runOpMode(){

        Servo OuttakeClaw = hardwareMap.servo.get("Outtake Claw");

        OuttakeClaw.setPosition(0);

        waitForStart();
        while(opModeIsActive()){
            if (gamepad1.b && !OuttakeClawClosed && ClawTime.seconds() >= .3){
                ClawTime.reset();
                OuttakeClaw.setPosition(.5);
                OuttakeClawClosed = true;
            }
            else if (gamepad1.b && OuttakeClawClosed && ClawTime.seconds() >= .3){
                ClawTime.reset();
                OuttakeClaw.setPosition(0);
                OuttakeClawClosed = false;
            }
        }
    }
}
