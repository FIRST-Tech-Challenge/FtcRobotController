package org.firstinspires.ftc.teamcode.OpModes;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.ejml.simple.SimpleMatrix;
import org.firstinspires.ftc.teamcode.Mechanisms.Drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.Mechanisms.Robot.Robot;

import java.util.ArrayList;
import java.util.List;

public class TeleopWithActions extends OpMode {
    private FtcDashboard dash = FtcDashboard.getInstance();
    private List<Action> runningActions = new ArrayList<>();

    Drivetrain drivetrain = null;
    FtcDashboard dashboard;
    Robot robot;
    @Override
    public void init() {
        robot = new Robot(hardwareMap);
        dashboard = FtcDashboard.getInstance();
        drivetrain = robot.drivetrain;
    }

    @Override
    public void loop() {
        TelemetryPacket packet = new TelemetryPacket();

        // updated based on gamepads
        runningActions.add(
                new Action() {
                    @Override
                    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                        double y = -gamepad1.left_stick_y;
                        double x = -gamepad1.left_stick_x;
                        double rx = gamepad1.right_stick_x;

                        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
                        double frontLeftPower = (y - x + rx) / denominator;
                        double backLeftPower = (y + x + rx) / denominator;
                        double frontRightPower = (y + x - rx) / denominator;
                        double backRightPower = (y - x - rx) / denominator;

                        SimpleMatrix drivePowers = new SimpleMatrix(
                                new double[][]{
                                        new double[]{frontLeftPower},
                                        new double[]{backLeftPower},
                                        new double[]{backRightPower},
                                        new double[]{frontRightPower}
                                }
                        );
                        drivetrain.setPower(drivePowers);
                        return true;
                    }
                }
        );
        // update running actions
        List<Action> newActions = new ArrayList<>();
        for (Action action : runningActions) {
            action.preview(packet.fieldOverlay());
            if (action.run(packet)) {
                newActions.add(action);
            }
        }
        runningActions = newActions;

        dash.sendTelemetryPacket(packet);
    }
}
