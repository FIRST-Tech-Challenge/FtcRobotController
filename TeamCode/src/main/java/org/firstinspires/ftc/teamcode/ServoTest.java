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

        servo = hardwareMap.get(Servo.class, "PixelReleaseServo");
        while(opModeIsActive()) {
            SetServoPosition(0.0, 3000);
            SetServoPosition(0.25, 1000);
            SetServoPosition(0.5, 1000);
            SetServoPosition(0.75, 1000);
            SetServoPosition(1.0,2000);
        }
    }

    private void SetServoPosition(double position, int sleepMilliseconds)
    {
        servo.setPosition(position);
        telemetry.addData("Servo Position: ", servo.getPosition());
        telemetry.update();
        sleep(sleepMilliseconds);
    }
}
