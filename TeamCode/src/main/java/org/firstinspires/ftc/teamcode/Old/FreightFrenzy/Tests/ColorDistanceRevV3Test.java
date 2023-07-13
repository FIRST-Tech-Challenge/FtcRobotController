package org.firstinspires.ftc.teamcode.Old.FreightFrenzy.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Old.FreightFrenzy.Components.Misc.ColorDistanceRevV3;
@Disabled

@Autonomous(name="ColorDistanceRevV3 Test")

public class ColorDistanceRevV3Test extends LinearOpMode {

    @Override
    public void runOpMode() {

        float hsvValues[] = {0F, 0F, 0F};
        ColorDistanceRevV3 colorDist = new ColorDistanceRevV3();

        // wait for the start button to be pressed.
        waitForStart();

        // loop and read the RGB and distance data.
        // Note we use opModeIsActive() as our loop condition because it is an interruptible method.
        while (opModeIsActive()) {
            // send the info back to driver station using telemetry function.
            telemetry.addData("Alpha", colorDist.alpha());
            telemetry.addData("Red  ", colorDist.red());
            telemetry.addData("Green", colorDist.green());
            telemetry.addData("Blue ", colorDist.blue());
            hsvValues = colorDist.hsvVal();
            telemetry.addData("Hue", hsvValues[0]+" "+hsvValues[1]+" "+hsvValues[2]);
            telemetry.addData("range", String.format("%.01f in", colorDist.getSensorDistance()));
            telemetry.update();

        }

    /* WORKING COLOR SENSOR V2
           //Configure V2 sensor on any I2C bus name it as "color". Save config. SCAN for change
            ColorSensor color;

            // Get the color sensor from hardwareMap
            color = hardwareMap.get(ColorSensor.class, "color");

            // Wait for the Play button to be pressed
            waitForStart();

            // While the Op Mode is running, update the telemetry values.
            while (opModeIsActive()) {
                telemetry.addData("Red", color.red());
                telemetry.addData("Green", color.green());
                telemetry.addData("Blue", color.blue());
                telemetry.update();
            }
    */


    }


}
