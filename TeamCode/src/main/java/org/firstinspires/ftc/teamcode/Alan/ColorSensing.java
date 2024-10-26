package org.firstinspires.ftc.teamcode.Alan;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;

@Disabled
@TeleOp
public class ColorSensing extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        ColorSensor cSensor = hardwareMap.colorSensor.get("ColorS");
        waitForStart();
        while(opModeIsActive()) {
            telemetry.addData("Red:", cSensor.red());
            telemetry.addData("Blue:", cSensor.blue());
            telemetry.addData("Green:", cSensor.green());
            telemetry.update();
        }
    }
}
