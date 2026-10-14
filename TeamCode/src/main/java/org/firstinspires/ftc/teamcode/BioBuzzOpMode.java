package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class BioBuzzOpMode extends LinearOpMode {
    private final MecanumDrive driveTrain = new MecanumDrive(this);
    private static final String TELEOP = "TELEOP";

    @Override
    public void runOpMode() {

        // Init
        driveTrain.init(hardwareMap, telemetry);

        // On initilization the Driver Station will prompt for which OpMode should be run - Auto Blue, Auto Red, or TeleOp
        while (opModeInInit()) {
            telemetry.update();
        }

        waitForStart();

        doTeleOp();
    }

    private void doTeleOp() {
        while (opModeIsActive()) {
            driveTrain.driveWithGamepad(gamepad1);
        }
    }
}
