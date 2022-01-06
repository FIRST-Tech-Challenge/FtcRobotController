package org.firstinspires.ftc.team6220_2021;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Park", group = "Autonomous")
public class Park extends MasterOpMode{

    @Override
    public void runOpMode() {
        Initialize();

        servoGrabber.setPosition(0.0);
        pauseMillis(500);
        servoArm.setPosition(0.81);
        waitForStart();
        motorArm.setTargetPosition(550);
        motorArm.setPower(0.9);
        pauseMillis(500);
        motorBL.setPower(0.8);
        motorBR.setPower(0.8);
        motorFL.setPower(0.8);
        motorFR.setPower(0.8);
        pauseMillis(1200);
        motorBL.setPower(-0.1);
        motorBR.setPower(-0.1);
        motorFL.setPower(-0.1);
        motorFR.setPower(-0.1);
        pauseMillis(100);
        motorArm.setTargetPosition(-10);
        motorArm.setPower(0.9);
        pauseMillis(500);
    }
}