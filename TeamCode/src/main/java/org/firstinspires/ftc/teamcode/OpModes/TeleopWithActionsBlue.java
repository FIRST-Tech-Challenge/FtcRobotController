package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware.Sensors.Battery;
import org.firstinspires.ftc.teamcode.Mechanisms.Arm.Arm;
import org.firstinspires.ftc.teamcode.Mechanisms.Claw.Claw;
import org.firstinspires.ftc.teamcode.Hardware.Sensors.Color;
import org.firstinspires.ftc.teamcode.Mechanisms.Drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.Mechanisms.Extension.Extension;
import org.firstinspires.ftc.teamcode.Mechanisms.Intake.Intake;
import org.firstinspires.ftc.teamcode.Mechanisms.Lift.Lift;
import org.firstinspires.ftc.teamcode.Mechanisms.Robot.Robot;
import org.firstinspires.ftc.teamcode.Mechanisms.Sweeper.Sweeper;

import java.util.HashMap;
import java.util.Map;


@Config
@TeleOp
public class TeleopWithActionsBlue extends OpMode {

    private FtcDashboard dash = FtcDashboard.getInstance();
    private Map<String, Action> runningActions = new HashMap<>();

    Drivetrain drivetrain = null;
    FtcDashboard dashboard;
    Robot robot;
    Arm arm;
    Claw claw;
    Color colorSensor;
    Extension extension;
    Lift lift;
    Battery battery;
    Sweeper sweeper;
    boolean firstRun = true;
    boolean red = false;
    public static double slowMultiplier = 0.25;
    public ElapsedTime timer = new ElapsedTime();
    boolean forceOuttake = false;

    @Override
    public void init() {
        dashboard = FtcDashboard.getInstance();
        robot = new Robot(hardwareMap);
        battery = robot.battery;
        drivetrain = robot.drivetrain;
        arm = robot.arm;
        claw = robot.claw;
        colorSensor = robot.colorSensor;
        extension = robot.extension;
        lift = robot.lift;
        sweeper = robot.sweeper;
    }

    @Override
    public void loop() {
        TelemetryPacket packet = new TelemetryPacket();
        if (firstRun) {
            runningActions.put("extension", extension.servoExtension(Extension.extensionState.RETRACT));
            runningActions.put("claw", claw.servoClaw(Claw.clawState.OPEN));
            runningActions.put("intake", robot.intakeMove(Intake.intakeState.STOP));
            runningActions.put("arm", arm.armRetract());
            firstRun = false;
        } else {
            if (forceOuttake){
                if (timer.seconds()>2){
                    timer.reset();
                    forceOuttake = false;
                } else {
                    runningActions.put("intake", robot.intakeMove(Intake.intakeState.OUTTAKE));
                }
            }
            if (gamepad1.right_trigger > 0.5) {
                runningActions.put("extension", extension.servoExtension(Extension.extensionState.EXTEND));
            }
            if (gamepad1.left_trigger > 0.5) {
                runningActions.put("extension", extension.servoExtension(Extension.extensionState.RETRACT));
            }
            if (gamepad2.left_bumper) {
                runningActions.put("claw", claw.servoClaw(Claw.clawState.OPEN));
            }
            if (gamepad2.right_bumper) {
                runningActions.put("claw", claw.servoClaw(Claw.clawState.CLOSE));
            }
            if (gamepad1.left_bumper) {
                if ((red&&!colorSensor.isBlue())||(!red&&!colorSensor.isRed())){
                    runningActions.put("intake", robot.intakeMove(Intake.intakeState.INTAKE));
                } else {
                    runningActions.put("intake", robot.intakeMove(Intake.intakeState.OUTTAKE));
                    forceOuttake = true;
                    timer.reset();
                }
            } else if (gamepad1.right_bumper) {
                runningActions.put("intake", robot.intakeMove(Intake.intakeState.OUTTAKE));
            } else {
                runningActions.put("intake", robot.intakeMove(Intake.intakeState.STOP));
            }
            if (gamepad2.cross) {
                runningActions.put("arm", arm.armSpecimen());
            }
            if (gamepad2.triangle) {
                runningActions.put("arm", arm.armSample());
            }
            if (gamepad1.triangle) {
                runningActions.put("sweep", sweeper.sweep());
            }
            if (Math.abs(gamepad2.left_stick_y) > 0.2) {
                runningActions.put("manualLift", lift.manualControl(-gamepad2.left_stick_y));
            } else {
                runningActions.put("manualLift", lift.manualControl(0));
            }
            if (gamepad1.cross) {
                runningActions.put(
                        "manualDrive",
                        drivetrain.manualControl(-gamepad1.left_stick_x * slowMultiplier, gamepad1.left_stick_y * slowMultiplier, gamepad1.right_stick_x * slowMultiplier)
                );
            } else {
                runningActions.put(
                        "manualDrive",
                        drivetrain.manualControl(-gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x)
                );
            }
            // update running actions
            HashMap<String, Action> newActions = new HashMap<>();
            for (Map.Entry<String, Action> entry : runningActions.entrySet()) {
                entry.getValue().preview(packet.fieldOverlay());
                if (entry.getValue().run(packet)) {
                    newActions.put(entry.getKey(), entry.getValue());
                }
            }
            runningActions = newActions;

            dash.sendTelemetryPacket(packet);
        }
    }
}
