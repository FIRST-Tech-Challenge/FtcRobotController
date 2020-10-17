package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Test Range Sensor", group="Linear Opmode")

public class TestSensor extends LinearOpMode {
    Robot robot;

    public void runOpMode () {
        robot = new Robot(this, true);
        robot.initIMU();

        while(!isStarted()) {
            telemetry.addData("range", String.format("%.01f cm", robot.getRange(false)));
            telemetry.update();

            //throttle to 10Hz loop to avoid burning excess CPU cycles for no reason
            sleep(100);
        }

        robot.driveController.driveWithRange(Vector2d.FORWARD, 10, true, true, 0.7, 0.4, this);
    }
}
