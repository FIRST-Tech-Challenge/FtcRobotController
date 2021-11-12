package org.firstinspires.ftc.teamcode.temp;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.competition.utils.StandardServo;

@Autonomous(name="Servotest", group="tests")

public class ServoOpMode extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        StandardServo servo = new StandardServo(hardwareMap, "servoTest1");
        servo.setPosition(0);
        waitForStart();
        resetStartTime();
        servo.setPosition(100);
        while(opModeIsActive());
    }

}
