package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
@Autonomous(name="testColorSensor")
public class TestColorSensor extends LinearOpMode {

    RobotClass robot;

    @Override
    public void runOpMode() throws InterruptedException {

        robot = new RobotClass(hardwareMap, telemetry, this);

        waitForStart();

        ColorSensor colorSensor= robot.colorSensor;

        while (opModeIsActive()) {

            telemetry.addData("Light Level: ", colorSensor.alpha());
            telemetry.addData("Red", colorSensor.red());
            telemetry.addData("Blue", colorSensor.blue());
            telemetry.addData("Green", colorSensor.green());
            telemetry.addData("RGB", colorSensor.argb());
            telemetry.update();
        }

        colorSensor.alpha();
    }
}
