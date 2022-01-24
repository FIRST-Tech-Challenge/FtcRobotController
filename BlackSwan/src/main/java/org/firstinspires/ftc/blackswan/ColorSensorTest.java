package org.firstinspires.ftc.blackswan;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;

public class ColorSensorTest extends LinearOpMode {

        ColorSensor colorSensorLeft;
        RevColorSensorV3 colorSensorRight;
        RevColorSensorV3 colorSensorBack;

    @Override
    public void runOpMode() throws InterruptedException {
        colorSensorLeft = hardwareMap.get(RevColorSensorV3.class,"colorSensorLeft");
        colorSensorRight = hardwareMap.get(RevColorSensorV3.class,"colorSensorRight");
        colorSensorBack = hardwareMap.get(RevColorSensorV3.class,"colorSensorBack");

        waitForStart();

        while (opModeIsActive()) {

            telemetry.addData("BackBlueValue: ",colorSensorBack.blue());
            telemetry.addData("BackRedValue: ",colorSensorBack.red());
            telemetry.addData("BackGreenValue: ",colorSensorBack.green());
            telemetry.update();
        }
    }
}
