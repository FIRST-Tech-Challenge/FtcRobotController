package org.firstinspires.ftc.teamcode.autonomous;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ScheduleCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.systems.SignalSystem;

@Autonomous(name = "Move to Signal Zone")
public class AutoSignal extends CommandOpMode {
    AutoPhase phase = AutoPhase.Waiting;
    
    SignalSystem signalSystem;

    @Override
    public void initialize() {
        signalSystem = new SignalSystem(hardwareMap, "Webcam 1");
        
        schedule(
                new InstantCommand(() -> signalSystem.startCamera()),
                new WaitUntilCommand(this::isStarted),
                new WaitUntilCommand(signalSystem::isResultReady),
                new ScheduleCommand(
                        new InstantCommand(() -> {
                            telemetry.addLine("Detected signal " + signalSystem.GetResult().name());
                            telemetry.update();

                            signalSystem.stopCamera();
                        })
                )
        );
    }
}
