package org.firstinspires.ftc.teamcode.tatooine.Opmodes.Teleops;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.tatooine.SubSystem.Intake;
import org.firstinspires.ftc.teamcode.tatooine.utils.Alliance.CheckAlliance;
import org.firstinspires.ftc.teamcode.tatooine.utils.gamepads.EasyGamepad;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name = "IntakeTestTeleOp", group = "TeleOp")
public class IntakeTestTeleOp extends LinearOpMode {
    private final FtcDashboard dash = FtcDashboard.getInstance();
    private List<Action> runningActions = new ArrayList<>();
    private final EasyGamepad easyGamepad1 = new EasyGamepad(gamepad1);
    private final boolean isRed = CheckAlliance.isRed();

    @Override
    public void runOpMode() throws InterruptedException {
        while (opModeIsActive()) {
            easyGamepad1.update(gamepad1);
            Intake intake = new Intake(this, isRed, true);
            TelemetryPacket packet = new TelemetryPacket();
            List<Action> newActions = new ArrayList<>();
            for (Action action : runningActions) {
                action.preview(packet.fieldOverlay());
                if (action.run(packet)) {
                    newActions.add(action);
                }
            }
            runningActions = newActions;
            if (easyGamepad1.getR1()) {
                runningActions.add(intake.intakeByColor(true));
            }
            if (easyGamepad1.getL1()) {
                runningActions.add(intake.intakeByColor(false));
            }
        }
    }
}
