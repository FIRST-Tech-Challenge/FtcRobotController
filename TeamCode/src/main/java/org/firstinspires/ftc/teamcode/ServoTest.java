package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name="Autonomous ServoTest", group="Linear Opmode")
public class ServoTest extends LinearOpMode {
    Servo servo;
    @Override
    public void runOpMode() {

        waitForStart();
        telemetry.addData("Entering servo test", "Servo0");

        servo = hardwareMap.get(Servo.class, "servo");
        while(opModeIsActive()) {
            servo.setPosition(0);
            sleep(1000);
            servo.setPosition(0.5);
            sleep(1000);
            servo.setPosition(1);
            sleep(1000);
            telemetry.addData("Servo Position", servo.getPosition());
            telemetry.addData("Status", "Running");
            telemetry.update();
        }
    }
}
