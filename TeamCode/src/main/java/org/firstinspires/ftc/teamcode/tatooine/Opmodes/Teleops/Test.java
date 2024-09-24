package org.firstinspires.ftc.teamcode.tatooine.Opmodes.Teleops;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Actions;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.tatooine.SubSystem.Intake;
import org.firstinspires.ftc.teamcode.tatooine.utils.gamepads.EasyGamepad;

import java.util.ArrayList;
import java.util.List;

public class Test extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        FtcDashboard dash = FtcDashboard.getInstance();
        List<Action> runningActions = new ArrayList<>();
        Intake intake = new Intake(hardwareMap);
        EasyGamepad easyGamepad1 = new EasyGamepad(gamepad1);
        EasyGamepad easyGamepad2 = new EasyGamepad(gamepad2);
        TelemetryPacket packet = new TelemetryPacket();
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(new Vector2d(0,0),0));
        while (opModeIsActive()){
            easyGamepad1.update(gamepad1);
            easyGamepad2.update(gamepad2);

            List<Action> newActions = new ArrayList<>();
            for (Action action : runningActions) {
                action.preview(packet.fieldOverlay());
                if (action.run(packet)) {
                    newActions.add(action);
                }
            }
            runningActions = newActions;
            dash.sendTelemetryPacket(packet);

            drive.fieldDrive(new Pose2d(new Vector2d(easyGamepad1.getLeftStickXPower(),easyGamepad2.getLeftStickYPower()),easyGamepad1.getRightTrigger()-easyGamepad1.getLeftTrigger()));
            if (gamepad1.a) {
                runningActions.add(
                        intake.intake(easyGamepad1.getRightBumper())
                );
            }

        }
    }
        }

