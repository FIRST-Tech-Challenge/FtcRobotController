package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


@TeleOp (name = "touchSensorTest", group = "sensors9073")

public class touchSensor extends LinearOpMode{
    TouchSensor touch;
    DcMotor motor;

    @Override
    public void runOpMode() {
        touch = hardwareMap.get(TouchSensor.class, "Touch");
        //loop while opMode runs
        waitForStart();
        while (opModeIsActive()) {
            if (touch.isPressed()) {
                boolean x = true;
                telemetry.addData("Touch Sensor", x);
            }
            else{
                boolean x = false;
                telemetry.addData("Touch Sensor", x);
            }
            telemetry.update();
        }
    }
}
