package org.firstinspires.ftc.teamcode.other;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name="AServo", group="linear")
public class AServo extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        resetStartTime();
        Servo servo = hardwareMap.get(Servo.class, "aServo");
        servo.setPosition(1);
        sleep(10000);
        servo.setPosition(0);
        while(opModeIsActive());
    }

}
