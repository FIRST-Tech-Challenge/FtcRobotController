package org.firstinspires.ftc.teamcode.auto;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;



@Autonomous(name="Blue Far Park Forward")
public class BlueFarParkFore extends CommandOpMode {
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
        park = UnworkingTrajectories.blueParkFarForward;
        switch(tagId) {
            case 0:
                park = UnworkingTrajectories.blueParkFarForward;
                break;
            case 1:
                park = UnworkingTrajectories.blueParkFarForward;
                break;
            case 2:
                park = UnworkingTrajectories.blueParkFarForward;
                break;
            default:
                park = UnworkingTrajectories.blueParkFarForward;
        }

        schedule(new SequentialCommandGroup ( //Makes the following code run one after another, like normal
                new TrajectorySequenceCommand(drive, park)
                //add ur code here, look back at last year if u need help
            )
        );
    }
}