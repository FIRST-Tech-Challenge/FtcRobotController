package org.firstinspires.ftc.teamcode.Mechanisms.Pivot.Tuners;


import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Mechanisms.Claw.Claw;
import org.firstinspires.ftc.teamcode.Mechanisms.Drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.Mechanisms.Pivot.Pivot;

@Config
@Autonomous(name = "Test Pivot", group = "Autonomous")
public class TunePivot extends LinearOpMode {
    Drivetrain drivetrain = null;

    @Override
    public void runOpMode() {
        Pivot pivot = new Pivot(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad2.y){
                Actions.runBlocking(pivot.flippyFlip());
            }
        }
    }
}
