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
import java.util.HashSet;
import java.util.List;


@Config
@TeleOp
public class TeleopWithActions extends OpMode {

    private FtcDashboard dash = FtcDashboard.getInstance();
    private HashSet<Action> runningActions = new HashSet<>();

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
        manageAction(extension.servoExtension(gamepad1.right_trigger > 0.2 ? Extension.extensionState.EXTEND : Extension.extensionState.RETRACT));
        manageAction(claw.servoClaw(gamepad2.left_bumper ? Claw.clawState.OPEN : gamepad2.right_bumper ? Claw.clawState.CLOSE : null));
        manageAction(robot.intakeMove(gamepad1.left_bumper ? Intake.intakeState.INTAKE : gamepad1.right_bumper ? Intake.intakeState.OUTTAKE : Intake.intakeState.STOP));
        manageAction(gamepad2.cross ? arm.servoArmSpec() : null);
        manageAction(gamepad2.triangle ? arm.servoArm() : null);
        manageAction(lift.manualControl(Math.abs(gamepad2.left_stick_y) > 0.2 ? -gamepad2.left_stick_y : 0));
        manageAction(drivetrain.manualControl(-gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x));

        //if(gamepad2.right_stick_y >= 0){
        //}
        // updated based on gamepads
        runningActions.add(
                drivetrain.manualControl(-gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x)
        );
        // update running actions
        HashSet<Action> newActions = new HashSet<>();
        for (Action action : runningActions) {
            action.preview(packet.fieldOverlay());
            if (action.run(packet)) {
                newActions.add(action);
            }
        }
        runningActions = newActions;

        dash.sendTelemetryPacket(packet);
    }
    private void manageAction(Action newAction) {
        if (newAction == null) return;

        // Prevent duplicate actions
        if (runningActions.contains(newAction)) return;

        // Check for conflicting actions
        runningActions.removeIf(existingAction -> isConflicting(existingAction, newAction));

        // Add the new action
        runningActions.add(newAction);
    }

    private boolean isConflicting(Action action1, Action action2) {
        // Define conflict logic here. For example:
        // Actions controlling the same mechanism (e.g., extension, claw, intake) should not run simultaneously.
        return action1.getClass().equals(action2.getClass());
    }
}
