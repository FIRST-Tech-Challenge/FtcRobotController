package org.firstinspires.ftc.teamcode.configs;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.TouchSensor;

@TeleOp
public class Config_ContinuousServo extends LinearOpMode {

    // declare variables here
    private CRServo IntakeBox;

    @Override
    public void runOpMode() {
        {
            IntakeBox = hardwareMap.get(CRServo.class, "IntakeBox");


            waitForStart();


            while (opModeIsActive()) {
                // do op mode things here

                if (gamepad1.a) {
                    IntakeBox.setPower(0.5);
                    sleep(2000);
                }
                else if(gamepad1.b) {
                    IntakeBox.setPower(-0.5);
                    sleep(2000);
                }
                else {
                    IntakeBox.setPower(0);
                }
            }
        }

    }
}