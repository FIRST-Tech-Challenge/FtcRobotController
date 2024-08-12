package org.firstinspires.ftc.teamcode.opmodes.auto;

import org.firstinspires.ftc.teamcode.commands.BackdropHome;
import org.firstinspires.ftc.teamcode.commands.SlideToPosition;
import org.firstinspires.ftc.teamcode.constants.SubsystemConstants;
import org.firstinspires.ftc.teamcode.constants.VisionConstants;
import org.firstinspires.ftc.teamcode.opmodes.Robot;
import org.firstinspires.ftc.teamcode.vision.DetectorPipeline;
import org.rustlib.commandsystem.DelayCommand;
import org.rustlib.commandsystem.InstantCommand;
import org.rustlib.commandsystem.ParallelCommandGroup;
import org.rustlib.commandsystem.PurePursuitAutonomousCommand;
import org.rustlib.commandsystem.SequentialCommandGroup;
import org.rustlib.commandsystem.WaitCommand;
import org.rustlib.core.Auton;
import org.rustlib.drive.Field;
import org.rustlib.drive.FollowPathCommand;
import org.rustlib.drive.Path;
import org.rustlib.drive.Waypoint;
import org.rustlib.geometry.Pose2d;
import org.rustlib.geometry.Rotation2d;
import org.rustlib.rustboard.Rustboard;
import org.rustlib.utils.Future;
import org.rustlib.vision.OpenCVGameElementDetector;

public abstract class AutonomousContainer extends Robot implements Auton {
    protected static Pose2d blueLeftStartPosition = new Pose2d(58.944, 7.916899, new Rotation2d(Math.PI));
    protected static Pose2d blueRightStartPosition = blueLeftStartPosition.translateX(Field.tileLengthIn * 2);
    private OpenCVGameElementDetector gameElementDetector;
    protected PurePursuitAutonomousCommand blueLeftCommand = new PurePursuitAutonomousCommand(
            blueLeftStartPosition,
            new SequentialCommandGroup(
                    new FollowPathCommand(this::getPurplePlacePath, drive), // Drive to the spike mark
                    new InstantCommand(purplePixelPlacer::place), // Place the pixel
                    new WaitCommand(Rustboard.loadSavedDouble("wait_time_BL", 1000)), // Wait for the pixel to drop
                    new ParallelCommandGroup( // Begin driving away and retract the dropper arm
                            new SequentialCommandGroup(new WaitCommand(1000), new InstantCommand(purplePixelPlacer::retract)),
                            new FollowPathCommand(Path.load("to_backdrop_BL"), drive)),
                    new SlideToPosition(slide, SubsystemConstants.Slide.autoPlacePosition), // Move the slide to the correct position
                    new DelayCommand(slide::atTargetPosition, 2000), // Wait until the slide is close to the correct position
                    new BackdropHome(drive.getBase(), slide, placer, new Future<>(this::getYellowPlaceWaypoint), 2000, 500), // Home in on the backdrop
                    new InstantCommand(placer::open), // Drop the yellow pixel
                    new WaitCommand(750), // Wait for the pixel to drop
                    new ParallelCommandGroup( // Begin driving away and stow the slide
                            new SequentialCommandGroup(
                                    new WaitCommand(750),
                                    new SlideToPosition(slide, SubsystemConstants.Slide.stowedPosition)
                            ),
                            new FollowPathCommand(Path.load("park_BL"), drive)
                    )
            )
    );
    protected PurePursuitAutonomousCommand redRightCommand = blueLeftCommand.mirror();
    protected PurePursuitAutonomousCommand blueRightCommand = new PurePursuitAutonomousCommand(
            blueRightStartPosition,
            new SequentialCommandGroup(
                    new InstantCommand(() -> aprilTagCamera.disable()),
                    new FollowPathCommand(this::getPurplePlacePath, drive), // Drive to the spike mark
                    new InstantCommand(purplePixelPlacer::place), // Place the pixel
                    new WaitCommand(Rustboard.loadSavedDouble("wait_time_BR", 1000)), // Wait for the pixel to drop
                    new FollowPathCommand(Path.load("back_up"), drive),
                    new ParallelCommandGroup( // Begin driving away and retract the dropper arm
                            new SequentialCommandGroup(new WaitCommand(1000), new InstantCommand(purplePixelPlacer::retract)),
                            new FollowPathCommand(Path.load("to_backdrop_BR"), drive)),
                    new SlideToPosition(slide, SubsystemConstants.Slide.autoPlacePosition), // Move the slide to the correct position
                    new DelayCommand(slide::atTargetPosition, 3000), // Wait until the slide is close to the correct position
                    new BackdropHome(drive.getBase(), slide, placer, new Future<>(this::getYellowPlaceWaypoint), 2000, 500), // Home in on the backdrop
                    new InstantCommand(placer::open), // Drop the yellow pixel
                    new WaitCommand(750), // Wait for the pixel to drop
                    new ParallelCommandGroup( // Begin driving away and stow the slide
                            new SequentialCommandGroup(
                                    new WaitCommand(750),
                                    new SlideToPosition(slide, SubsystemConstants.Slide.stowedPosition)
                            )
                    )
            )
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
    public void onInit() {
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
