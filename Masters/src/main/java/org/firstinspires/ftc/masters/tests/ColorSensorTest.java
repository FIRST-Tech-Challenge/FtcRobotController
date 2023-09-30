package org.firstinspires.ftc.masters.tests;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class ColorSensorTest extends LinearOpMode {

    RevColorSensorV3 colorSensor;

    @Override
    public void runOpMode() throws InterruptedException {
        colorSensor = hardwareMap.get(RevColorSensorV3.class, "color sensor");
        waitForStart();
        while(opModeIsActive()) {
            telemetry.addData("Sensed Red", colorSensor.red());
            telemetry.addData("Sensed Blue", colorSensor.blue());
            telemetry.addData("Sensed Green", colorSensor.green());
            telemetry.update();
        }
    }
}
