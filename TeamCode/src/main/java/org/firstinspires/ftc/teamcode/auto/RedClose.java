package org.firstinspires.ftc.teamcode.auto;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.subsystems.Slides;

@Autonomous(name="Red Close")
public class RedClose extends CommandOpMode {
    //Add Motors and servos not for drivebase here
    SampleMecanumDrive drive;

    int tagId = 0; //Dw about it rn

    Slides s = new Slides(hardwareMap);

    @Override
    public void initialize() {
        drive = new SampleMecanumDrive(hardwareMap);
        //motors and servos hardwareMap stuff here

        while (!isStarted() && !isStopRequested()) {
            //dw about it
        }

        UnworkingTrajectories.generateTrajectories(drive); //Loads trajectories from trajectories file

        TrajectorySequence prop;
        TrajectorySequence score;
        TrajectorySequence park;

        switch(tagId) {
            case 0:
                prop = UnworkingTrajectories.propCloseLeftRed;
                score = UnworkingTrajectories.scoreCloseLeftRed;
                park = UnworkingTrajectories.parkCloseLeftRed;
                break;
            case 1:
                prop = UnworkingTrajectories.propCloseMidRed;
                score = UnworkingTrajectories.scoreCloseMidRed;
                park = UnworkingTrajectories.parkCloseMidRed;
                break;
            case 2:
                prop = UnworkingTrajectories.propCloseRightRed;
                score = UnworkingTrajectories.scoreCloseRightRed;
                park = UnworkingTrajectories.parkCloseRightRed;
                break;
            default:
                prop = UnworkingTrajectories.propCloseLeftRed;
                score = UnworkingTrajectories.scoreCloseLeftRed;
                park = UnworkingTrajectories.parkCloseLeftRed;
        }

        schedule(new SequentialCommandGroup ( //Makes the following code run one after another, like normal
                new TrajectorySequenceCommand(drive, prop),
                new TrajectorySequenceCommand(drive, score),
                new TrajectorySequenceCommand(drive, park)
                //add ur code here, look back at last year if u need help
                    //s.goToPosition(1500);
            )

        );
    }
}