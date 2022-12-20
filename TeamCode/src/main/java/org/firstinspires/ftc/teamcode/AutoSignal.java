package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.FunctionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SelectCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.autonomous.SignalState;
import org.firstinspires.ftc.teamcode.commands.DetectSignal;
import org.firstinspires.ftc.teamcode.commands.MoveTime;
import org.firstinspires.ftc.teamcode.systems.DriveSystem;
import org.firstinspires.ftc.teamcode.systems.SignalSystem;

import java.util.Timer;

@Autonomous(name = "Move to Signal Zone")
public class AutoSignal extends CommandOpMode {
    ElapsedTime timer;
    SignalSystem signalSystem;
    DriveSystem drive;

    @Override
    public void initialize() {
        ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        signalSystem = new SignalSystem(hardwareMap, "Webcam 1");
        drive = new DriveSystem(hardwareMap);
        
        schedule(
            new DetectSignal(signalSystem),
            new WaitUntilCommand(this::isStarted),
            new InstantCommand(() -> {
                telemetry.addLine("Signal State: " + signalSystem.GetResult().name());
                telemetry.update();
            }),
            new SelectCommand(this::chooseCommand)
        );
    }
    
    private Command chooseCommand() {
        double speed = 0.3;
        double time = 1500;
        
        switch (signalSystem.GetResult()) {
            case One:
                return new MoveTime(timer, drive, -speed, 0, 0, time);
            case Three:
                return new MoveTime(timer, drive, speed, 0, 0, time);
            default:
                return new MoveTime(timer, drive, 0, 0, 0, 0);
        }
    }
}
