package org.firstinspires.ftc.teamcode.temp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Servotest", group="tests")

public class ServoOpMode extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        resetStartTime();
        Servo servo = hardwareMap.get(Servo.class, "servo");
        Servo servoTwo = hardwareMap.get(Servo.class, "servo2");
        servo.resetDeviceConfigurationForOpMode();
        servoTwo.resetDeviceConfigurationForOpMode();
        servo.setPosition(1.0);
        servoTwo.setPosition(1.0);
        while(opModeIsActive());
    }

}
