package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.commands.DetectSignal;
import org.firstinspires.ftc.teamcode.commands.UpdatePose;
import org.firstinspires.ftc.teamcode.systems.OdometrySystem;
import org.firstinspires.ftc.teamcode.systems.SignalSystem;

@Autonomous(name = "Move to Signal Zone")
public class AutoSignal extends CommandOpMode {
    ElapsedTime elapsedTime;
    
    SignalSystem signalSystem;
    OdometrySystem odometrySystem;
    Pose2d start;

    @Override
    public void initialize() {
        elapsedTime = new ElapsedTime();
        
        signalSystem = new SignalSystem(hardwareMap, "Webcam 1");
        odometrySystem = new OdometrySystem(hardwareMap, start, elapsedTime);
        
        schedule(
            new UpdatePose(odometrySystem),
            new SequentialCommandGroup(
                new DetectSignal(signalSystem),
                new InstantCommand(() -> {
                    telemetry.addLine("Signal State: " + signalSystem.GetResult().name());
                    telemetry.update();
                })
            ));
    }
}
