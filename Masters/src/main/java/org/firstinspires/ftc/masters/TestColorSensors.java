package org.firstinspires.ftc.masters;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
@TeleOp(name = "Test color sensors", group ="test")
public class TestColorSensors extends LinearOpMode {

    RobotClass robot;

    @Override
    public void runOpMode() throws InterruptedException {

        robot = new RobotClass(hardwareMap, telemetry, this);

        waitForStart();

        while (opModeIsActive()) {

            telemetry.addData("Left Light Level: ", robot.colorSensorLeft.alpha());
            telemetry.addData("Left Red", robot.colorSensorLeft.red());
            telemetry.addData("Left Blue", robot.colorSensorLeft.blue());
            telemetry.addData("Left Green", robot.colorSensorLeft.green());
            telemetry.addData("Left RGB", robot.colorSensorLeft.argb());

            telemetry.addData("Middle Light Level: ", robot.colorSensorMiddle.alpha());
            telemetry.addData("Middle Red", robot.colorSensorMiddle.red());
            telemetry.addData("Middle Blue", robot.colorSensorMiddle.blue());
            telemetry.addData("Middle Green", robot.colorSensorMiddle.green());
            telemetry.addData("Middle RGB", robot.colorSensorMiddle.argb());

            telemetry.addData("Right Light Level: ", robot.colorSensorRight.alpha());
            telemetry.addData("Right Red", robot.colorSensorRight.red());
            telemetry.addData("Right Blue", robot.colorSensorRight.blue());
            telemetry.addData("Right Green", robot.colorSensorRight.green());
            telemetry.addData("Right RGB", robot.colorSensorRight.argb());

            telemetry.update();
        }
    }
}
