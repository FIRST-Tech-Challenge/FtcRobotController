package org.firstinspires.ftc.teamcode.auto;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Disabled
@Autonomous(name="Red Far")
public class RedFar extends CommandOpMode {
    //Add Motors and servos not for drivebase here
    SampleMecanumDrive drive;

    int tagId = 0; //Dw about it rn

    @Override
    public void initialize() {
        drive = new SampleMecanumDrive(hardwareMap);
        //motors and servos hardwareMap stuff here

        while (!isStarted() && !isStopRequested()) {
            //dw about it
        }

        UnworkingTrajectories.generateTrajectories(drive); //Loads trajectories from trajectories file

        TrajectorySequence placeOnLine;
        TrajectorySequence park;
        park = UnworkingTrajectories.redParkFar;
        switch(tagId) {
            case 0:
                placeOnLine = UnworkingTrajectories.propFarLeftRed;
                break;
            case 1:
                placeOnLine = UnworkingTrajectories.propFarMidRed;
                break;
            case 2:
                placeOnLine = UnworkingTrajectories.propFarRightRed;
                break;
            default:
                placeOnLine = UnworkingTrajectories.propFarLeftRed;
        }

        schedule(new SequentialCommandGroup ( //Makes the following code run one after another, like normal
                new TrajectorySequenceCommand(drive, placeOnLine),
                new TrajectorySequenceCommand(drive, park)
                //add ur code here, look back at last year if u need help
            )
        );
    }
}