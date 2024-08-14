package org.firstinspires.ftc.teamcode.opmodes.auto;

import org.firstinspires.ftc.teamcode.constants.VisionConstants;
import org.firstinspires.ftc.teamcode.opmodes.Robot;
import org.firstinspires.ftc.teamcode.vision.DetectorPipeline;
import org.rustlib.commandsystem.PurePursuitAutonomousCommand;
import org.rustlib.commandsystem.SequentialCommandGroup;
import org.rustlib.core.AutonomousCore;
import org.rustlib.drive.Field;
import org.rustlib.drive.Path;
import org.rustlib.drive.Waypoint;
import org.rustlib.geometry.Pose2d;
import org.rustlib.geometry.Rotation2d;
import org.rustlib.vision.OpenCVGameElementDetector;

public abstract class AutonomousContainer extends Robot implements AutonomousCore {
    protected static Pose2d blueLeftStartPosition = new Pose2d(58.944, 7.916899, new Rotation2d(Math.PI));
    protected static Pose2d blueRightStartPosition = blueLeftStartPosition.translateX(Field.tileLengthIn * 2);
    private OpenCVGameElementDetector gameElementDetector;
    protected PurePursuitAutonomousCommand blueLeftCommand = new PurePursuitAutonomousCommand(
            blueLeftStartPosition,
            new SequentialCommandGroup()
    );
    protected PurePursuitAutonomousCommand redRightCommand = blueLeftCommand.mirror();
    protected PurePursuitAutonomousCommand blueRightCommand = new PurePursuitAutonomousCommand(
            blueRightStartPosition,
            new SequentialCommandGroup()
    );
    protected PurePursuitAutonomousCommand redLeftCommand = blueRightCommand.mirror();

    Path getPurplePlacePath() {
        switch (gameElementDetector.getElementLocation()) {

        }
        return new Path();
    }

    Waypoint getYellowPlaceWaypoint() {
        switch (gameElementDetector.getElementLocation()) {

        }
        return null;
    }

    @Override
    public void opModeInit() {
        gameElementDetector = OpenCVGameElementDetector.getBuilder()
                .setHardwareMap(hardwareMap)
                .setCameraName("game element detector cam")
                .setStreamSize(OpenCVGameElementDetector.StreamDimension.HIGH_DEF)
                .setDetectorPipeline(new DetectorPipeline())
                .setFrameAveragingCount(VisionConstants.elementDetectionFrameAverageCount)
                .closePipelineOnOpModeStart()
                .build();
    }
}
