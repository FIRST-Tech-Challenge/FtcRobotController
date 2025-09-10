package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Subsystems.Drive;
import org.firstinspires.ftc.teamcode.Tools.Mouse;

public class Robot extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {
        Drive driveSubsystem = new Drive("drive", hardwareMap);
        waitForStart();

        while (opModeIsActive()) {

            driveSubsystem.FeildCentric(gamepad1);

            telemetry.addData("X", Mouse.getX());
            telemetry.addData("Y", Mouse.getY());
            telemetry.addData("Theta", Mouse.getTheta());
            telemetry.update();
        }
    }
}
