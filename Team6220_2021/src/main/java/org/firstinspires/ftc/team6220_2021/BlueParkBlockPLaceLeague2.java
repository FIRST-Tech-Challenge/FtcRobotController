package org.firstinspires.ftc.team6220_2021;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Disabled
@Autonomous(name = "BlueBlockPlace", group = "Autonomous")
public class BlueParkBlockPLaceLeague2 extends MasterOpMode{

    @Override
    public void runOpMode() {
        Initialize();

        servoGrabber.setPosition(0.0);
        pauseMillis(500);
        servoArm.setPosition(0.81);
        waitForStart();
        servoArm.setPosition(0.15);
        motorArm.setTargetPosition(900);
        motorArm.setPower(0.9);
        pauseMillis(500);
        motorBL.setPower(0.6);
        motorBR.setPower(0.6);
        motorFL.setPower(0.6);
        motorFR.setPower(0.6);
        pauseMillis(650);
        motorBL.setPower(0);
        motorBR.setPower(0);
        motorFL.setPower(0);
        motorFR.setPower(0);
        pauseMillis(100);
        motorBL.setPower(0.1);
        motorBR.setPower(0.1);
        motorFL.setPower(0.1);
        motorFR.setPower(0.1);
        pauseMillis(900);
        motorBL.setPower(0);
        motorBR.setPower(0);
        motorFL.setPower(0);
        motorFR.setPower(0);
        pauseMillis(100);
        servoGrabber.setPosition(0.7);
        pauseMillis(700);
        motorBL.setPower(-0.3);
        motorBR.setPower(-0.3);
        motorFL.setPower(-0.3);
        motorFR.setPower(-0.3);
        pauseMillis(500);
        motorBL.setPower(0);
        motorBR.setPower(0);
        motorFL.setPower(0);
        motorFR.setPower(0);
        pauseMillis(100);
        motorBL.setPower(-0.3);
        motorBR.setPower(0.3);
        motorFL.setPower(-0.3);
        motorFR.setPower(0.3);
        pauseMillis(1350);
        motorBL.setPower(0);
        motorBR.setPower(0);
        motorFL.setPower(0);
        motorFR.setPower(0);
        pauseMillis(100);
        motorBL.setPower(0.8);
        motorBR.setPower(0.8);
        motorFL.setPower(0.8);
        motorFR.setPower(0.8);
        pauseMillis(1500);
        motorBL.setPower(-0.1);
        motorBR.setPower(-0.1);
        motorFL.setPower(-0.1);
        motorFR.setPower(-0.1);
        pauseMillis(100);
        servoGrabber.setPosition(0.0);
        pauseMillis(100);
        servoArm.setPosition(0.81);
        motorArm.setTargetPosition(-10);
        motorArm.setPower(0.9);
        pauseMillis(700);
    }
}