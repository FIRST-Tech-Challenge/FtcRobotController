package org.firstinspires.ftc.teamcode.swerveAuto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.teamUtil.*;
import org.firstinspires.ftc.teamcode.teamUtil.trajectoryAssembly.*;

@Autonomous(name="Swerve Auto Test", group="Worlds")
public class exampleAuto extends LinearOpMode {
    robotConfig r;
    ElapsedTime elapsedTime = new ElapsedTime();

    @Override
    public void runOpMode() {
        telemetry.addLine("initilising");
        telemetry.update();
        r = robotConfig.getInstance(this);
        r.initSystems(robotConstants.configuredSystems.BOTH_MODULES);
        telemetry.addLine("initilised");
        telemetry.update();

        pose2D startPose = new pose2D(new coordinate2D(0,0), new angle(90, angle.angleType.ABSOLUTE));

        r.swerve.setStartPose2D(startPose);

        trajectoryAssembly testDrive = new trajectoryAssembly()
                .addOffsetActionMarker(10, () -> {
                    telemetry.addLine("hey");
                })
                .addSegment(new pose2D(new coordinate2D(1000, 0), new angle(90)))
                .addSegment(new pose2D(new coordinate2D(1000, 1000), new angle(90)))
                .addSegment(new pose2D(new coordinate2D(0, 1000), new angle(90)))
                .addOffsetActionMarker(1, () -> {
                    telemetry.addLine("hey");
                })
                .addSegment(new pose2D(new coordinate2D(0, 0), new angle(90)))

                .build();

        waitForStart();
        robotConfig.elapsedTime.reset();

        r.swerve.setTrajectoryAssembly(testDrive);
        elapsedTime.reset();
        while (opModeIsActive()){
            r.encoderRead.encoderBulkRead();
            telemetry.update();
            r.swerve.trajectoryFollowerUpdate();
        }

        r.closeLogs();
    }
}
