package org.firstinspires.ftc.team6220_2020.TestClasses;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "LauncherTest", group = "TeleOp")
public class LauncherTest extends LinearOpMode {

    DcMotor motorFrontLeft = hardwareMap.dcMotor.get("motorLauncher");

    motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    @Override
    public void runOpMode() throws InterruptedException {


        idle();
    }
}
