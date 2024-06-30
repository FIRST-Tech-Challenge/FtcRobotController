package org.firstinspires.ftc.masters.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp(name = "Analog Servo Test", group = "test")
public class analogServoTest extends LinearOpMode {

    Servo axon;
    AnalogInput axonAnalog;

    @Override
    public void runOpMode() {

        axon = hardwareMap.servo.get("axon0");
        axonAnalog = hardwareMap.get(AnalogInput.class, "axonAnalog0");
        axon.setPosition(1);

        waitForStart();

        while (opModeIsActive()){

            if (gamepad1.a){
                axon.setPosition(0);
            }
            if (gamepad1.b) {
                axon.setPosition(1);
            }

            double position = axonAnalog.getVoltage() / 3.3 * 360;
            telemetry.addData("Analog", position);
            telemetry.update();

        }

    }

}
