package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Core.Drivetrain;
import org.firstinspires.ftc.teamcode.Core.MechanumHardwareMap;
import org.firstinspires.ftc.teamcode.Core.Pinpoint;

@TeleOp(group = "Drivetrain", name = "Drivetrain Test")

public class DrivetrainTest extends LinearOpMode {

    public Drivetrain drivetrain;
    public Pinpoint pinpoint;
    public MechanumHardwareMap mechanumHardwareMap;

    @Override
    public void runOpMode() {

        mechanumHardwareMap = new MechanumHardwareMap(hardwareMap);
        pinpoint = new Pinpoint(mechanumHardwareMap);
        drivetrain = new Drivetrain(telemetry, gamepad1, mechanumHardwareMap);

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.x) {
                pinpoint.resetIMU();
            }

            drivetrain.update();
            pinpoint.update();
            telemetry.addData("Raw x", pinpoint.getX());
            telemetry.addData("Raw y", pinpoint.getY());
            telemetry.addData("Heading", pinpoint.getHeading());
            telemetry.update();
        }
    }
}
