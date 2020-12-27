package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drivebase.GyroSensor;
import org.firstinspires.ftc.teamcode.drivebase.MecanumDrivebase;

@TeleOp(name = "Vector Mecanum Drive", group = "Linear Opmode")
//@Disabled
public class MecanumTeleop extends LinearOpMode {

    private MecanumDrivebase mecanumDrivebase = new MecanumDrivebase();
    private GyroSensor gyroSensor = new GyroSensor();

    @Override
    public void runOpMode() {

        mecanumDrivebase.initialize(this);
        gyroSensor.initialize(this);

        // Wait for the start button
        waitForStart();

        mecanumDrivebase.startControl();
        gyroSensor.startControl();

        while(opModeIsActive()) {

            mecanumDrivebase.readController(gamepad1);
            gyroSensor.updateAngles(this);
            mecanumDrivebase.setGyroAngle(gyroSensor.getDirection());
            mecanumDrivebase.whileOpModeIsActive(this);

            mecanumDrivebase.addTelemetry(telemetry);
            telemetry.update();
            idle();

        }

        mecanumDrivebase.stop();
    }

}