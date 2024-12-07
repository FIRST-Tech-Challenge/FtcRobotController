package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Hardware.Sensors.Battery;
import org.firstinspires.ftc.teamcode.Mechanisms.Arm.Arm;
import org.firstinspires.ftc.teamcode.Mechanisms.Claw.Claw;
import org.firstinspires.ftc.teamcode.Mechanisms.Drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.Mechanisms.Extension.Extension;
import org.firstinspires.ftc.teamcode.Mechanisms.Intake.Intake;
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
    Intake intake;
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
        Intake intake = new Intake(hardwareMap);
    }

    @Override
    public void loop() {
        TelemetryPacket packet = new TelemetryPacket();
        if(gamepad1.right_trigger > 0){
            runningActions.add(extension.servoExtension());
        }
        if(gamepad1.left_trigger > 0){
            runningActions.add(extension.servoExtension());
        }
        if(gamepad2.left_bumper){
            runningActions.add(claw.servoClaw());
        }
        if(gamepad2.right_bumper){
            runningActions.add(claw.servoClaw());
        }
        if(gamepad1.left_bumper){
            runningActions.add(intake.motorIntake());
        }
        if(gamepad1.right_bumper){
            runningActions.add(intake.motorIntake());
        }
        if(gamepad1.circle){
            runningActions.add(arm.servoArm());
        }
        if(gamepad1.cross){
            runningActions.add(arm.servoArm());
        }
        if(gamepad2.left_stick_y != 0){
            runningActions.add(lift.manualControl());
        }
        if(gamepad2.right_stick_y >= 0){
        }

        // updated based on gamepads
        runningActions.add(
                drivetrain.manualControl()
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
