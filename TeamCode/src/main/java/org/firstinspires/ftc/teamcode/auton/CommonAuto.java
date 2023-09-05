package org.firstinspires.ftc.teamcode.auton;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drivePose.SampleMecanumFaster;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous
@Config
public class CommonAuto extends LinearOpMode {
    SampleMecanumFaster drive;
    private final FtcDashboard dashboard = FtcDashboard.getInstance();
    private final Telemetry dashboardTelemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
    ElapsedTime timer = new ElapsedTime();
    double lastTime = 0;
    double currentTime = 0;
    public static double cycleDelay = .52;
    private boolean isAutoEnd = false;

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumFaster(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.setPoseEstimate(AutoConstants.L_START);

        TrajectorySequence path = drive.trajectorySequenceBuilder(AutoConstants.L_START)
                .addTemporalMarker(0, () -> {
//                    turret.setTargetAngle(TURRET_SCORE_ANG);
                })
                .lineToLinearHeading(AutoConstants.L_SCORE_POSE)
//                .UNSTABLE_addTemporalMarkerOffset(turretAfterScoreDelay, () -> {
//                    turret.setTargetAngle(TURRET_PICK_ANG);
//                })
                .waitSeconds(cycleDelay)
                // END CONE 1 (PRELOAD)
                .lineTo(AutoConstants.L_SCORE_VECTOR)
                .build();


        TrajectorySequence leftPark = drive.trajectorySequenceBuilder(path.end())
                .setVelConstraint(AutoConstants.PARK_VEL)
                .setAccelConstraint(AutoConstants.PARK_ACCEL)
//                .addTemporalMarker(0, () -> {
//                    turret.setTargetAngle(0);
//                })
                .lineToLinearHeading(AutoConstants.L_PARK_LEFT)
                .waitSeconds(1.5)
                .back(15.0)
                .build();


        telemetry.setMsTransmissionInterval(50);

        while (!isStarted() && !isStopRequested()) {
            dashboardTelemetry.addLine("Robot is (!isStarted() && !isStopRequested())");
            dashboardTelemetry.update();
            sleep(20);
        }

        drive.followTrajectorySequenceAsync(path);

        lastTime = timer.milliseconds();

        while (!isStopRequested() && opModeIsActive()) {
            drive.update();
            currentTime = timer.milliseconds();
            dashboardTelemetry.addLine("loop-time: " + currentTime);
            if (currentTime > 7000.0 && !isAutoEnd) {
                drive.followTrajectorySequenceAsync(leftPark);
                isAutoEnd = true;
            }
            dashboardTelemetry.update();
        }

    }
}
