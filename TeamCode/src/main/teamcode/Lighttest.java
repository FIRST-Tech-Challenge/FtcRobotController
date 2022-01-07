package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

//@TeleOp(name = "Lighttest")

public class Lighttest extends Taco_FF_Super_Class {

    @Override
    public void runOpMode() {
        initialization(false);
        waitForStart();
        while (opModeIsActive()) {
            Blinkyboi.setPattern(RevBlinkinLedDriver.BlinkinPattern.fromNumber(44));
            }
        }
    }
