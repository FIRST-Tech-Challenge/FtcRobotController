package org.firstinspires.ftc.teamcode.Tests;

import static com.qualcomm.robotcore.hardware.Servo.Direction.FORWARD;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Components.RFModules.Devices.RFCRServo;
import org.firstinspires.ftc.teamcode.Components.RFModules.Devices.RFDualServo;
import org.firstinspires.ftc.teamcode.Components.tseDepositor;

@Autonomous(name="RFDualServoTest")

public class RFDualServoTest extends LinearOpMode{
//    @Override

    LinearOpMode op;

    public void runOpMode() {
        RFDualServo dualservo = new RFDualServo("turretAngleAdjust1", "turretAngleAdjust2", FORWARD, op);
        dualservo.setPosition(0.9);
        sleep(5000);
        dualservo.setPosition(0.1);
    }
}

