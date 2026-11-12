package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.mechanisims.InTake;
import org.firstinspires.ftc.teamcode.mechanisims.LaunchSystem;
import org.firstinspires.ftc.teamcode.mechanisims.MecanumDrive;

@Autonomous
public class MainAutonomousMode extends OpMode {

    private static final double ROLL_FORWARD_TIME = 0.75;
    private static final double ROTATE_TIME_ROF = 0.5;
    private static final double STRAFE_TIME_LOF = 2.0;
    private static final double STRAFE_TIME_ROF = 0.75;
    private static final double ROLL_FORWARD_SPEED = 0.3;
    private static final double ROTATE_SPEED = 0.3;
    private static final double STRAFE_SPEED = 0.6;

    private final ElapsedTime timer = new ElapsedTime();

    private enum StartingPosition {
        LeftOfFlower, RightOfFlower

    }

    private int shotsToFire = 4;

    private enum State {
        RollForward,
        Launching,
        Rotating,
        Strafing,
        Complete
    }

    private StartingPosition startingPosition = StartingPosition.LeftOfFlower;
    private State state = State.RollForward;

    double launchSpeedMultiplier = 1.0;
    double intakeSpeedMultiplier = 1.0;
    LaunchSystem launchSystem;
    MecanumDrive mecanumDrive;

    InTake intake;


    @Override
    public void init() {
        launchSystem = new LaunchSystem(hardwareMap, HardwareNames.POLLEN_LAUNCH_MOTOR, HardwareNames.POLLEN_FEED_MOTOR_1, HardwareNames.POLLEN_FEED_MOTOR_2);
        mecanumDrive = new MecanumDrive(hardwareMap);
        intake = new InTake(hardwareMap);

    }

    @Override
    public void init_loop() {
        upDateSettings();
    }

    @Override
    public void start() {
        timer.reset();
    }

    @Override
    public void loop() {
        telemetry.addData("Current state", state);
        telemetry.addData("Timer seconds", timer.seconds());
        switch (state) {
            case RollForward:
                mecanumDrive.drive(ROLL_FORWARD_SPEED, 0.0, 0.0);
                if (timer.seconds() >= ROLL_FORWARD_TIME) {
                    mecanumDrive.stop();
                    timer.reset();
                    state = State.Launching;
                }
                break;

            case Launching:
                launchSystem.update(1.0);
                intake.update(1.0);
                if (launchSystem.isReady()) {
                    if (shotsToFire > 0) {
                        shotsToFire--;
                        launchSystem.requestLaunch();
                    } else {
                        launchSystem.stopMotorIfIdle();
                        intake.stop();
                        state = State.Rotating;
                        timer.reset();
                    }
                }
                break;

            case Rotating:
                mecanumDrive.drive(0.0, 0.0, ROTATE_SPEED);
                if (startingPosition == StartingPosition.LeftOfFlower || timer.seconds() >= ROTATE_TIME_ROF) {
                    mecanumDrive.stop();
                    state = State.Strafing;
                    timer.reset();
                }
                break;

            case Strafing:
                mecanumDrive.drive(0.0, STRAFE_SPEED, 0.0);
                double secondsToStrafe = startingPosition == StartingPosition.LeftOfFlower ? STRAFE_TIME_LOF : STRAFE_TIME_ROF;
                if (timer.seconds() >= secondsToStrafe) {
                    mecanumDrive.stop();
                    timer.reset();
                    state = State.Complete;
                }
                break;

            case Complete:
                break;
        }

    }

    private void upDateSettings() {
        if (gamepad1.dpadUpWasPressed()) {
            launchSpeedMultiplier += 0.05;
        }
        if (gamepad1.dpadDownWasPressed()) {
            launchSpeedMultiplier -= 0.05;
        }
        telemetry.addData("change launchspeed", "dpad up/down");
        telemetry.addData("launchSpeedMultiplier", launchSpeedMultiplier);


        if (gamepad1.dpadRightWasPressed()) {
            intakeSpeedMultiplier += 0.05;
        }
        if (gamepad1.dpadLeftWasPressed()) {
            intakeSpeedMultiplier -= 0.05;
        }
        telemetry.addData("change intakespeed", "dpad left/right");
        telemetry.addData("intakeSpeedMultiplier", intakeSpeedMultiplier);

        if (gamepad1.left_stick_button) {
            startingPosition = startingPosition == StartingPosition.LeftOfFlower ? StartingPosition.RightOfFlower : StartingPosition.LeftOfFlower;
        }
        telemetry.addData("toggle starting position", "press left stick");
        telemetry.addData("curent starting position", startingPosition);
    }

}
