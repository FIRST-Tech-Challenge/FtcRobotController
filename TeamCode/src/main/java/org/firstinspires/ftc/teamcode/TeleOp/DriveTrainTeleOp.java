package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;

@TeleOp
public class DriveTrainTeleOp extends LinearOpMode{

    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        Drivetrain drivetrain = new Drivetrain(this);

        while (opModeIsActive()) {
            drivetrain.teleOp();
        }
    }
}

