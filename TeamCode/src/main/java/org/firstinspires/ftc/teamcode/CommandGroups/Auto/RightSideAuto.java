package org.firstinspires.ftc.teamcode.CommandGroups.Auto;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import org.firstinspires.ftc.teamcode.CommandGroups.ArmPositions.ArmStowHigh;
import org.firstinspires.ftc.teamcode.CommandGroups.ArmPositions.SpecimenPlacePos;
import org.firstinspires.ftc.teamcode.CommandGroups.AutomatedMovements.PlaceSpecimenAddOffset;
import org.firstinspires.ftc.teamcode.CommandGroups.AutomatedMovements.Sweep1;
import org.firstinspires.ftc.teamcode.CommandGroups.AutomatedMovements.Sweep2;
import org.firstinspires.ftc.teamcode.CommandGroups.AutomatedMovements.WallPickUp;
import org.firstinspires.ftc.teamcode.Commands.Claw.CloseClaw;
import org.firstinspires.ftc.teamcode.Commands.Drive.FollowPath;
import org.firstinspires.ftc.teamcode.Commands.Drive.MoveToPose;
import org.firstinspires.ftc.teamcode.Commands.Pause;
import org.firstinspires.ftc.teamcode.RobotContainer;
import org.firstinspires.ftc.teamcode.Subsystems.LinearSlide.SlideTargetHeight;
import org.firstinspires.ftc.teamcode.utility.AutoFunctions;

import java.util.ArrayList;

// Example Sequential Command Group
// There are also:
// ParallelCommandGroup
// ParallelRaceGroup
// ParallelDeadlineGroup

public class RightSideAuto extends SequentialCommandGroup {

    // constructor
    public RightSideAuto() {
        // start pos (0.25, 1.6, -90) on field
        addCommands (
                // sets the starting position
                new InstantCommand(() -> RobotContainer.odometry.setCurrentPos(new Pose2d(-0.22, 1.57, new Rotation2d(Math.toRadians(-90))))),
                //makes sure the claw is closed
                new CloseClaw(),

                new Pause(0.25),

                new PlaceSpecimenAddOffset(),

                new ArmStowHigh(),

                new Sweep1(),

                new WallPickUp(),

                new PlaceSpecimenAddOffset(),

                new MoveToPose(
                    2.0,
                    1.5,
                    AutoFunctions.redVsBlue(new Pose2d(-1.2, 1.2, new Rotation2d(Math.toRadians(-90.0))))
                )

        );


    }

}