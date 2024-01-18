package org.firstinspires.ftc.teamcode.auto;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;



@Autonomous(name="Red Close Park Forward")
public class RedCloserParkFore extends CommandOpMode {
    //Add Motors and servos not for drivebase here
    SampleMecanumDrive drive;

    int tagId = 0; //Dw about it rn

    @Override
    public void initialize() {

        schedule(new BulkCacheCommand(hardwareMap));
        drive = new SampleMecanumDrive(hardwareMap);
        //motors and servos hardwareMap stuff here

        UnworkingTrajectories.generateTrajectories(drive); //Loads trajectories from trajectories file

//        TrajectorySequence placeOnLine;
        TrajectorySequence testing;
        testing = UnworkingTrajectories.testing;

        while (opModeInInit()) {
            telemetry.update();
        }


//        switch(tagId) {
//            case 0:
//                park = UnworkingTrajectories.redParkCloseForward;
//                break;
//            case 1:
//                park = UnworkingTrajectories.redParkCloseForward;
//                break;
//            case 2:
//                park = UnworkingTrajectories.redParkCloseForward;
//                break;
//            default:
//                park = UnworkingTrajectories.redParkCloseForward;
//        }

        schedule(new SequentialCommandGroup ( //Makes the following code run one after another, like normal
                new TrajectorySequenceCommand(drive, testing)
                //add ur code here, look back at last year if u need help
            )
        );


    }
}