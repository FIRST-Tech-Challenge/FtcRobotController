package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.shooter.ShooterPID1Encoder;


@TeleOp(name="Shooter RPM with Helper", group="Linear Opmode")
//@Disabled
public class ShooterRPMWithHelper extends LinearOpMode {

private ShooterPID1Encoder shooterControls = new ShooterPID1Encoder();

    @Override
    public void runOpMode() {

        if (shooterControls.initialize(this)) {
            telemetry.addData("Status", "Initialized");
        } else {
            telemetry.addData("Status", "Initialization Error");
        }

        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        shooterControls.startControl();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            shooterControls.readController(gamepad1);
            shooterControls.whileOpModeIsActive();

            shooterControls.addTelemetry(telemetry);
            telemetry.update();
            idle();
        }
        shooterControls.stop();
    }
}
