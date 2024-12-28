package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Hardware.Sensors.Battery;
import org.firstinspires.ftc.teamcode.Mechanisms.Arm.Arm;
import org.firstinspires.ftc.teamcode.Mechanisms.Claw.Claw;
import org.firstinspires.ftc.teamcode.Mechanisms.Drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.Mechanisms.Extension.Extension;
import org.firstinspires.ftc.teamcode.Mechanisms.Intake.Intake;
import org.firstinspires.ftc.teamcode.Mechanisms.Lift.Lift;
import org.firstinspires.ftc.teamcode.Mechanisms.Robot.Robot;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;


@Config
@TeleOp
public class TeleopWithActions extends OpMode {

    private FtcDashboard dash = FtcDashboard.getInstance();
    private Map<String, Action> runningActions = new HashMap<>();

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
        battery = new Battery(hardwareMap);
        arm = new Arm(hardwareMap);
        claw = new Claw(hardwareMap);
        extension = new Extension(hardwareMap);
        lift = new Lift(hardwareMap, battery);
        intake = new Intake(hardwareMap);
    }

    @Override
    public void loop() {
        TelemetryPacket packet = new TelemetryPacket();
        if (gamepad1.right_trigger > 0.5){
                runningActions.put("extension.extensionState", extension.servoExtension(Extension.extensionState.EXTEND));
        }
        if (gamepad1.left_trigger>0.5){
            runningActions.put("extension.extensionState", extension.servoExtension(Extension.extensionState.RETRACT));
        }
        if (gamepad2.left_bumper){
            runningActions.put("claw.ClawState", claw.servoClaw(Claw.clawState.OPEN));
        }
        if (gamepad2.right_bumper){
            runningActions.put("claw.ClawState", claw.servoClaw(Claw.clawState.CLOSE));
        }
        if (gamepad1.left_bumper){
            runningActions.put("intake.intakeState", robot.intakeMove(Intake.intakeState.INTAKE));
        } else if (gamepad1.right_bumper){
            runningActions.put("intake.intakeState", robot.intakeMove(Intake.intakeState.OUTTAKE));
        } else {
            runningActions.put("intake.intakeState", robot.intakeMove(Intake.intakeState.STOP));
        }
        if(gamepad2.cross) { runningActions.put("arm.servoArm", arm.servoArmSpec());}
        if(gamepad2.triangle) {runningActions.put("arm.servoArm", arm.servoArm());}
        if (Math.abs(gamepad2.left_stick_y) > 0.2) {
            runningActions.put("lift.manualControl", lift.manualControl(gamepad2.left_stick_y));
        } else {
            runningActions.put("lift.manualControl", lift.manualControl(0));
        }
        runningActions.put(
                "drivetrain.manualControl",
                drivetrain.manualControl(-gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x)
        );
        // update running actions
        HashMap<String, Action> newActions = new HashMap<>();
        for (Map.Entry<String,Action> entry : runningActions.entrySet()) {
            entry.getValue().preview(packet.fieldOverlay());
            if (entry.getValue().run(packet)) {
                newActions.put(entry.getKey(), entry.getValue());
            }
        }
        runningActions = newActions;

        dash.sendTelemetryPacket(packet);
    }
}
