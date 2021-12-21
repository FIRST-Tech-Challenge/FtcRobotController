package org.firstinspires.ftc.teamcode.main.autonomous.sensors.other;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.competition.utils.interactions.items.StandardIMU;

import java.util.Dictionary;

@TeleOp(name = "IMU Test")
public class IMUTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        StandardIMU imu = new StandardIMU(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {
            Dictionary<StandardIMU.DataPoint, Float> data = imu.getData();

            telemetry.addData("Heading", data.get(StandardIMU.DataPoint.HEADING));
            telemetry.addData("Roll", data.get(StandardIMU.DataPoint.ROLL));
            telemetry.addData("Pitch", data.get(StandardIMU.DataPoint.PITCH));

            telemetry.update();
        }
    }
}
