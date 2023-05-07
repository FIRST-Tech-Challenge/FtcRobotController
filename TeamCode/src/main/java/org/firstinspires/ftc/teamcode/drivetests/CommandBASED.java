package org.firstinspires.ftc.teamcode.drivetests;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.HoldElevator;
import org.firstinspires.ftc.teamcode.commands.MoveElevator;
import org.firstinspires.ftc.teamcode.subsystems.ElevatorSubsystem;

import ftc.rogue.blacksmith.BlackOp;
import ftc.rogue.blacksmith.Scheduler;
import ftc.rogue.blacksmith.listeners.ReforgedGamepad;

@Config
@TeleOp(name="Command Based", group="Linear Opmode")
public class CommandBASED extends BlackOp {

    @Override
    public void go() {

        ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem(hardwareMap);
        elevatorSubsystem.setDefaultCommand(new HoldElevator(elevatorSubsystem));

        MoveElevator eleHigh = new MoveElevator(elevatorSubsystem, 1000);

        ReforgedGamepad driver = new ReforgedGamepad(gamepad1);

        waitForStart();

        Scheduler.launchOnStart(this, () -> {

            driver.dpad_up.onRise(() -> eleHigh.schedule());
            driver.dpad_down.onRise(() -> new MoveElevator(elevatorSubsystem, 0).schedule());

            elevatorSubsystem.update();

            mTelemetry().addData("eleTarget", elevatorSubsystem.getEleTarget());
            mTelemetry().addData("elePos", elevatorSubsystem.getElePos());
            mTelemetry().addData("DcMotorPos", elevatorSubsystem.getLeftDcMotorElePos());
//            mTelemetry().addData("MotorPos", elevatorSubsystem.getLeftMotorElePos());
            mTelemetry().addData("elePower", elevatorSubsystem.getElePower());
            mTelemetry().update();
        });
    }
}
