package org.firstinspires.ftc.teamcode.commands;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.dragonswpilib.command.CommandBase;
import org.firstinspires.ftc.teamcode.subsystems.AprilTagsDetection_SS;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

import org.firstinspires.ftc.dragonswpilib.command.CommandBase;

public class TagDetection_CMD extends CommandBase {

    private final AprilTagsDetection_SS mAprilTagsDetection_SS;
    private final Telemetry mTelemetry;
    public int tagValue = 0;

    public TagDetection_CMD(Telemetry telemetry, AprilTagsDetection_SS pADetectionTags){
        mTelemetry = telemetry;
        mAprilTagsDetection_SS = pADetectionTags;


        addRequirements(pADetectionTags);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        int aprilTagDetected  = mAprilTagsDetection_SS.readTags();
        switch (aprilTagDetected) {
            case 1:
                tagValue = 1;
                break;
            case 2:
                tagValue = 2;
                break;
            case 3:
                tagValue = 3;
                break;
        }


    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {

    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return tagValue== 1 || tagValue== 2 || tagValue== 3 ;
    }
}
