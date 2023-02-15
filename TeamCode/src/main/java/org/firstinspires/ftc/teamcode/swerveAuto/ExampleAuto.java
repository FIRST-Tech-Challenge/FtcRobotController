package org.firstinspires.ftc.teamcode.swerveAuto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.teamUtil.*;
import org.firstinspires.ftc.teamcode.teamUtil.trajectoryAssembly.*;

@Autonomous(name="Swerve Auto Test", group="Worlds")
public class ExampleAuto extends LinearOpMode {
    RobotConfig r;
    ElapsedTime elapsedTime = new ElapsedTime();

    @Override
    public void runOpMode() {
        telemetry.addLine("initilising");
        telemetry.update();
        r = RobotConfig.getInstance(this);
        r.initSystems(RobotConstants.configuredSystems.BOTH_MODULES);
        telemetry.addLine("initilised");
        telemetry.update();

        Pose2D startPose = new Pose2D(new Coordinate2D(0,0), new Angle(90, Angle.angleType.ABSOLUTE));

        r.swerve.setStartPose2D(startPose);

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

        r.swerve.setTrajectoryAssembly(testDrive);
        elapsedTime.reset();
        while (opModeIsActive()){
            r.encoderRead.encoderBulkRead();

            r.swerve.trajectoryFollowerUpdate();
        }

        r.closeLogs();
    }
}
