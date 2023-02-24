package org.firstinspires.ftc.teamcode.swerveAuto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.SwerveDriveBase;
import org.firstinspires.ftc.teamcode.teamUtil.*;
import org.firstinspires.ftc.teamcode.teamUtil.trajectoryAssembly.*;

/**
 * This is outdated lmao
 */
@Disabled
@Autonomous(name="Swerve Auto Test", group="Worlds")
public class ExampleAuto extends LinearOpMode {
    RobotConfig r;
    ElapsedTime elapsedTime = new ElapsedTime();

    SwerveDriveBase swerve;

    @Override
    public void runOpMode() {
        telemetry.addLine("initilising");
        telemetry.update();
        r = RobotConfig.getInstance(this);
        r.initSystems(RobotConstants.configuredSystems.BOTH_MODULES);
        telemetry.addLine("initilised");
        telemetry.update();

        swerve = r.getSubsystem(RobotConstants.configuredSystems.BOTH_MODULES);

        Pose2D startPose = new Pose2D(new Coordinate2D(0,0), new Angle(90, Angle.angleType.ABSOLUTE));

        swerve.setStartPose2D(startPose);

        TrajectoryAssembly testDrive = new TrajectoryAssembly()
                .addOffsetActionMarker(10, () -> {
                    telemetry.addLine("hey");
                })
                .addSegment(new Pose2D(new Coordinate2D(1000, 0), new Angle(90)))
                .addSegment(new Pose2D(new Coordinate2D(1000, 1000), new Angle(90)))
                .addSegment(new Pose2D(new Coordinate2D(0, 1000), new Angle(90)))
                .addOffsetActionMarker(1, () -> {
                    telemetry.addLine("hey");
                })
                .addSegment(new Pose2D(new Coordinate2D(0, 0), new Angle(90)))

                .build();

        waitForStart();
        RobotConfig.elapsedTime.reset();

        swerve.setTrajectoryAssembly(testDrive);
        elapsedTime.reset();
        while (opModeIsActive()){
            //encoderRead.encoderBulkRead();

            swerve.trajectoryFollowerUpdate();
        }

        r.closeLogs();
    }
}
