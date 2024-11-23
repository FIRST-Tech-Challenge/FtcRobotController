package org.firstinspires.ftc.teamcode.OpModes;

import static org.firstinspires.ftc.teamcode.Mechanisms.Claw.Claw.clawState.CLOSE;
import static org.firstinspires.ftc.teamcode.Mechanisms.Claw.Claw.clawState.OPEN;
import static org.firstinspires.ftc.teamcode.Mechanisms.Extension.Extension.extensionState.EXTEND;
import static org.firstinspires.ftc.teamcode.Mechanisms.Extension.Extension.extensionState.RETRACT;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.ejml.simple.SimpleMatrix;
import org.firstinspires.ftc.teamcode.Hardware.Sensors.Battery;
import org.firstinspires.ftc.teamcode.Mechanisms.Arm.Arm;
import org.firstinspires.ftc.teamcode.Mechanisms.Claw.Claw;
import org.firstinspires.ftc.teamcode.Mechanisms.Drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.Mechanisms.Extension.Extension;
import org.firstinspires.ftc.teamcode.Mechanisms.Lift.Lift;
import org.firstinspires.ftc.teamcode.Mechanisms.Robot.Robot;

import java.util.ArrayList;
import java.util.List;

public class TeleopWithActions extends OpMode {
    private FtcDashboard dash = FtcDashboard.getInstance();
    private List<Action> runningActions = new ArrayList<>();

    Drivetrain drivetrain = null;
    FtcDashboard dashboard;
    Robot robot;
    Arm arm;
    Claw claw;
    Extension extension;
    Lift lift;
    Battery battery;
    @Override
    public void init() {
        robot = new Robot(hardwareMap);
        dashboard = FtcDashboard.getInstance();
        drivetrain = robot.drivetrain;
        Battery battery = new Battery(hardwareMap);
        Arm arm = new Arm(hardwareMap);
        Claw claw = new Claw(hardwareMap);
        Extension extension = new Extension(hardwareMap);
        Lift lift = new Lift(hardwareMap, battery);
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

                        if(gamepad1.right_trigger > 0){
                            runningActions.add(extension.servoExtension(RETRACT));
                        }
                        if(gamepad1.left_trigger > 0){
                            runningActions.add(extension.servoExtension(EXTEND));
                        }
                        if(gamepad2.left_bumper){
                            runningActions.add(claw.servoClaw(OPEN));
                        }
                        if(gamepad2.right_bumper){
                            runningActions.add(claw.servoClaw(CLOSE));
                        }

//                        if(gamepad2.right_stick_y > 0){
//                            runningActions.add(arm.servoArm());
//                        }
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
