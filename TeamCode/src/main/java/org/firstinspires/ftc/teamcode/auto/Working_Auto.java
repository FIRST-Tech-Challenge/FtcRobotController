package org.firstinspires.ftc.teamcode.auto;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous(name="ActuallyWorkingAuto")
public class Working_Auto extends CommandOpMode {
    SampleMecanumDrive drive;

    @Override
    public void initialize() {
        drive = new SampleMecanumDrive(hardwareMap);

        while (!isStarted() && !isStopRequested()) {
            telemetry.addLine("Waiting For Start...");
        }

        UnworkingTrajectories.generateTrajectories(drive);

        schedule(new SequentialCommandGroup (
                        new TrajectorySequenceCommand(drive, UnworkingTrajectories.testing)
                )
        );
    }
}
