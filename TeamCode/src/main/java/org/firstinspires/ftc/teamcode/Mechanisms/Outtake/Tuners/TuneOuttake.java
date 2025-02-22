package org.firstinspires.ftc.teamcode.Mechanisms.Outtake.Tuners;

import org.firstinspires.ftc.teamcode.Mechanisms.Claw.Claw;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Mechanisms.Drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.Mechanisms.Outtake.Outtake;

@Config
@Autonomous(name = "Tune Outtake", group = "Autonomous")
public class TuneOuttake extends LinearOpMode {

    @Override
    public void runOpMode() {
        Outtake outtake = new Outtake(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {
            if(gamepad1.cross){
                Actions.runBlocking(outtake.OuttakeFrontSpecimen());
            }
            if(gamepad1.square){
                Actions.runBlocking(outtake.OuttakeBackSpecimen());
            }
            if(gamepad1.circle){
                Actions.runBlocking(outtake.OuttakeBackSample());
            }
        }
    }
}