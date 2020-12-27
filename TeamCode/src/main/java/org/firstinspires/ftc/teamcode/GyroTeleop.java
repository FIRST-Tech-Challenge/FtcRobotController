package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drivebase.GyroSensor;

@TeleOp(name = "Gyro Sensor Test", group = "Linear Opmode")
//@Disabled
public class GyroTeleop extends LinearOpMode {

    private GyroSensor gyroSensor = new GyroSensor();

    @Override
    public void runOpMode() {

        gyroSensor.initialize(this);

        // Wait for the start button
        waitForStart();

        gyroSensor.startControl();

        while(opModeIsActive()) {

            gyroSensor.updateAngles(this);

            telemetry.update();
            idle();

        }

        gyroSensor.stop();
    }

}