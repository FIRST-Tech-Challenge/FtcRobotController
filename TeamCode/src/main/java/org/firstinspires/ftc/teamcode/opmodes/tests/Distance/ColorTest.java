package org.firstinspires.ftc.teamcode.opmodes.tests.Distance;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.robot.TurtleRobotAuto;

import com.qualcomm.robotcore.hardware.ColorRangeSensor;

@Autonomous
public class ColorTest extends LinearOpMode {
    // Define a variable for our color sensor
    TurtleRobotAuto robot = new TurtleRobotAuto(this);
    ColorRangeSensor color;




    @Override
    public void runOpMode() {
        // Get the color sensor from hardwareMap
        color = hardwareMap.get(ColorRangeSensor.class, "Color");


        // Wait for the Play button to be pressed
        waitForStart();

        // While the Op Mode is running, update the telemetry values.
        while (opModeIsActive()) {
            telemetry.addData("Distance", color.getDistance(DistanceUnit.INCH));
            Distance();
            telemetry.addData("Red", color.red());
            telemetry.addData("Green", color.green());
            telemetry.addData("Blue", color.blue());
            telemetry.update();
        }

    }



    public void Distance() {
        color.getDistance(DistanceUnit.INCH);

    }
}