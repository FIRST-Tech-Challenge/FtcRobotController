package org.firstinspires.ftc.teamcode.CommandGroups.AutomatedMovements;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import org.firstinspires.ftc.teamcode.CommandGroups.ArmPositions.DropToGrab;
import org.firstinspires.ftc.teamcode.CommandGroups.ArmPositions.HuntingPos;
import org.firstinspires.ftc.teamcode.Commands.Claw.CloseClaw;
import org.firstinspires.ftc.teamcode.Commands.Claw.OpenClaw;
import org.firstinspires.ftc.teamcode.Commands.Claw.WaitForClawButton;
import org.firstinspires.ftc.teamcode.Commands.ConvertAngleForWristRotate;
import org.firstinspires.ftc.teamcode.Commands.Drive.MoveToPickup;
import org.firstinspires.ftc.teamcode.Commands.Pause;
import org.firstinspires.ftc.teamcode.RobotContainer;
import org.firstinspires.ftc.teamcode.utility.VisionProcessorMode;

// Example Sequential Command Group
// There are also:
// ParallelCommandGroup
// ParallelRaceGroup
// ParallelDeadlineGroup

public class AutoPickUpOffGround extends SequentialCommandGroup {

    // constructor
    public AutoPickUpOffGround() {

        addCommands (

                new HuntingPos(),

                new OpenClaw(),

                // Original location of ... moved down.
                // new ConvertAngleForWristRotate(),

                new Pause(0.25),

                new MoveToPickup(),

                // Moved here to move first to best vantage point.
                new ConvertAngleForWristRotate(),

                new Pause(0.25),

                new DropToGrab(),

                new WaitForClawButton(),

                new Pause(.25),

                new CloseClaw(),

                new HuntingPos(),

                new Pause(0.1)


        );
    }


    // This method is called once when command is finished.
    // if command either finishes or canceled, then switch off camera
    @Override
    public void end(boolean interrupted) {
        RobotContainer.clawCamera.setVisionProcessingMode(VisionProcessorMode.NONE);
    }


}