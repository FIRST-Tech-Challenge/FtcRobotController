package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drivebase.MecanumDrivebase;

@TeleOp(name = "Test Drivebase Encoders", group = "Linear Opmode")
//@Disabled
public class TestDrivebaseEncoder extends LinearOpMode {

    /* Declare OpMode members. */
    private MecanumDrivebase mecanumDrivebase = new MecanumDrivebase();

    public void runOpMode() {

        mecanumDrivebase.initialize(this);
        // Wait for the start button

        mecanumDrivebase.startControl();
        mecanumDrivebase.resetEncoders();

        waitForStart();

        while (opModeIsActive()) {
            mecanumDrivebase.drivebaseEncoders(telemetry);
            telemetry.update();
        }
    }
}