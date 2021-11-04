package org.firstinspires.ftc.teamcode.temp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.competition.utils.StandardServo;

@TeleOp(name="Servotest", group="tests")

public class ServoOpMode extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        StandardServo servo = new StandardServo(hardwareMap, "servoTest1");
        StandardServo servo2 = new StandardServo(hardwareMap, "servoTest2");
        StandardServo servo3 = new StandardServo(hardwareMap, "servoTest3");
        servo.setPosition(0);
        servo2.setPosition(0);
        servo3.setPosition(0);
        waitForStart();
        resetStartTime();
        servo.setPosition(-180);
        servo2.setPosition(180);
        while(opModeIsActive()) {
            sleep(2000);
            servo3.setPosition(180);
            sleep(200);
            servo.setPosition(180);
            servo2.setPosition(-180);
        }
    }

}
