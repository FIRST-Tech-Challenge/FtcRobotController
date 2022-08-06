package org.firstinspires.ftc.teamcode.Tests;

import static com.qualcomm.robotcore.hardware.Servo.Direction.FORWARD;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Components.RFModules.Devices.RFDualServo;
import org.firstinspires.ftc.teamcode.Components.tseDepositor;
import org.firstinspires.ftc.teamcode.Components.Logger;

@Autonomous(name="RFDualServoTest")

public class RFDualServoTest extends LinearOpMode{
//    @Override

    LinearOpMode op;
    Logger logger;

    public void runOpMode() {
        RFDualServo dualservo = new RFDualServo(FORWARD, op, logger);
        dualservo.setPosition(0.9);
        sleep(5000);
        dualservo.setPosition(0.1);
    }
}

