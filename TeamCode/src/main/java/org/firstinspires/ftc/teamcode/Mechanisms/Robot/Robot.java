package org.firstinspires.ftc.teamcode.Mechanisms.Robot;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SequentialAction;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Hardware.Sensors.Battery;
import org.firstinspires.ftc.teamcode.Mechanisms.Drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.Mechanisms.Intake.Intake;
import org.firstinspires.ftc.teamcode.Mechanisms.Pivot.Pivot;

public class Robot {
    public Drivetrain drivetrain;
    public Battery battery;
    public Pivot pivot;
    public Intake intake;
    public Robot(HardwareMap hardwareMap){
        this.battery = new Battery(hardwareMap);
        this.drivetrain = new Drivetrain(hardwareMap,battery);
        this.pivot = new Pivot(hardwareMap);
        this.intake = new Intake(hardwareMap);
    }
    public enum intakeMechState {
        INTAKE, STOP, OUTTAKE
    }
    public Action intakeMove(Intake.intakeState intakeMechState){
        return new SequentialAction(
                pivot.flippyFlip(intakeMechState),
                intake.motorIntake(intakeMechState)
        );
    }
}
