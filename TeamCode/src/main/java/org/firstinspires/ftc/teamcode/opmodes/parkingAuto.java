package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

import org.firstinspires.ftc.teamcode.*;


import org.firstinspires.ftc.teamcode.Robot;

@Autonomous(name="parkingAuto", group = "Real")
public class parkingAuto extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        Robot robot = new Robot(this);
        robot.configureAutoSetting();

        int allianceNumber = 1;

        Pose2d initialPose = new Pose2d(-3*Constants.INCH_TO_TILE*allianceNumber,-1*Constants.INCH_TO_TILE*allianceNumber,Math.toRadians(90)*allianceNumber);
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        TrajectoryActionBuilder parkObZone = drive.actionBuilder(initialPose)
                .strafeTo(new Vector2d(-2.6*Constants.INCH_TO_TILE*allianceNumber,-2.6*Constants.INCH_TO_TILE*allianceNumber));

        Action chosenAction = parkObZone.build();

        waitForStart();

        Actions.runBlocking(chosenAction);

        while (opModeIsActive() && !isStopRequested()) {
            robot.run();
        }
    }

}
