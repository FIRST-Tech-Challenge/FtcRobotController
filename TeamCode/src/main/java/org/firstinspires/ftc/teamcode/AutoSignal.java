package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SelectCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.autonomous.SignalState;
import org.firstinspires.ftc.teamcode.commands.DetectSignal;
import org.firstinspires.ftc.teamcode.commands.MoveForTime;
import org.firstinspires.ftc.teamcode.commands.UpdatePose;
import org.firstinspires.ftc.teamcode.systems.DriveSystem;
import org.firstinspires.ftc.teamcode.systems.OdometrySystem;
import org.firstinspires.ftc.teamcode.systems.SignalSystem;

import java.util.Set;

@Autonomous(name = "Move to Signal Zone")
public class AutoSignal extends CommandOpMode {
    ElapsedTime elapsedTime;
    
    SignalSystem signalSystem;
    OdometrySystem odometrySystem;
    DriveSystem drive;
    Pose2d start;

    @Override
    public void initialize() {
        elapsedTime = new ElapsedTime();
        
        signalSystem = new SignalSystem(hardwareMap, "Webcam 1");
        odometrySystem = new OdometrySystem(hardwareMap, start, elapsedTime);
        drive = new DriveSystem(hardwareMap, true);
        
        schedule(
            new UpdatePose(odometrySystem),
            new SequentialCommandGroup(
                new DetectSignal(signalSystem),
                new InstantCommand(() -> {
                    telemetry.addLine("Signal State: " + signalSystem.GetResult().name());
                    telemetry.update();
                }),
                    new MoveForTime(elapsedTime, drive, 0, RobotConfig.autoSpeed, RobotConfig.autoTimeF)
            )
        );
    }

    public Command chooseCommand() {
        SignalState state = signalSystem.GetResult();

        switch (state) {
            case One:
                return new MoveForTime(elapsedTime, drive, RobotConfig.autoSpeed, 0, RobotConfig.autoTimeS);
            case Three:
                return new MoveForTime(elapsedTime, drive, -RobotConfig.autoSpeed, 0, RobotConfig.autoTimeS);
        }

        return new CommandBase() {};
    }
}
