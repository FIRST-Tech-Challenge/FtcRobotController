package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.commands.DetectSignal;
import org.firstinspires.ftc.teamcode.systems.SignalSystem;

@Autonomous(name = "Move to Signal Zone")
public class AutoSignal extends CommandOpMode {
    SignalSystem signalSystem;

    @Override
    public void initialize() {
        signalSystem = new SignalSystem(hardwareMap, "Webcam 1");
        
        schedule(
            new SequentialCommandGroup(
                new DetectSignal(signalSystem),
                new InstantCommand(() -> {
                    telemetry.addLine("Detected signal " + signalSystem.GetResult().name());
                    telemetry.update();
                })
            )
        );
    }
}
