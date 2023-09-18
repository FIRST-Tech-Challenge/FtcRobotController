package org.firstinspires.ftc.teamcode.auton;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.command.MovePosition;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumFaster;
import org.firstinspires.ftc.teamcode.hardware.Turret;
import org.firstinspires.ftc.teamcode.subsystem.MyCamera;
import org.firstinspires.ftc.teamcode.subsystem.TestMotor;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous
@Config
public class CommonAuto extends LinearOpMode {
    SampleMecanumFaster drive;
    MyCamera myCamera;
    Turret turret;
    TestMotor testMotor;
    private final FtcDashboard dashboard = FtcDashboard.getInstance();
    private final Telemetry dashboardTelemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
    ElapsedTime timer = new ElapsedTime();
    double lastTime = 0;
    double currentTime = 0;
    public static double cycleDelay = .52;
    public static double turretAfterScoreDelay = .7;
    private boolean isAutoEnd = false;
    private Pose2d lastPose = new Pose2d(11.96, 0.4, 0.0);

    private enum Auto_State {
        drivePose, alignAprilTag, PARK, STOP
    }
    Auto_State mState = Auto_State.drivePose;

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumFaster(hardwareMap, dashboardTelemetry);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.setPoseEstimate(AutoConstants.L_START);

        myCamera = new MyCamera(hardwareMap, dashboardTelemetry);

//        turret = new Turret();
//        turret.init(hardwareMap, dashboardTelemetry);
//        turret.setTargetAngle(0);

        testMotor = new TestMotor(hardwareMap, dashboardTelemetry);

        TrajectorySequence path = drive.trajectorySequenceBuilder(AutoConstants.L_START)
//                .addTemporalMarker(0.5, () -> {
                    // This marker runs two seconds into the trajectory
//                    turret.setTargetAngle(100);
//                    testMotor.setSpeed(0.2);
                    // Run your action in here!
//                })
//                .addTemporalMarker(0.5, 0.1, () -> {
//                    // This example will run 50% of the way through the path, plus 0.1 seconds
//                    // The offset can be left at zero but is useful for making slight adjustments to the timing
//                })
//                .addDisplacementMarker(() -> {
//                    // This marker runs after the first splineTo()
//
//                    // Run your action in here!
//                })
//                .addDisplacementMarker(20, () -> {
//                    // This marker runs after robot drives 20 inches in the trajectory
//
//                    // Run your action in here!
//                })
//                .addSpatialMarker(new Vector2d(20, 20), () -> {
//                    // This marker runs at the point that gets
//                    // closest to the (20, 20) coordinate
//
//                    // Run your action in here!
//                })

                .lineToLinearHeading(AutoConstants.L_SCORE_POSE)
                .UNSTABLE_addTemporalMarkerOffset(turretAfterScoreDelay, () -> {
//                    turret.setTargetAngle(300);
                    mState = Auto_State.alignAprilTag;
                })
//                .waitSeconds(cycleDelay)
                // END CONE 1 (PRELOAD)
//                .lineTo(AutoConstants.L_SCORE_VECTOR)
                .build();

        TrajectorySequence path2 = drive.trajectorySequenceBuilder(lastPose)
                .setVelConstraint(AutoConstants.PARK_VEL)
                .setAccelConstraint(AutoConstants.PARK_ACCEL)
                .lineToLinearHeading(AutoConstants.L_PARK_LEFT)
                .UNSTABLE_addTemporalMarkerOffset(turretAfterScoreDelay, () -> {
//                    turret.setTargetAngle(300);
                })
                .waitSeconds(cycleDelay)
                .build();


        TrajectorySequence leftPark = drive.trajectorySequenceBuilder(path.end())
                .setVelConstraint(AutoConstants.PARK_VEL)
                .setAccelConstraint(AutoConstants.PARK_ACCEL)
                .addTemporalMarker(.5, () -> {
//                    turret.setTargetAngle(600);
                })
                .lineToLinearHeading(AutoConstants.L_PARK_LEFT)
                .waitSeconds(1.5)
                .build();

//        dashboardTelemetry.setMsTransmissionInterval(50);//Default value: 250

        while (!isStarted() && !isStopRequested()) {
            dashboardTelemetry.addLine("Robot is (!isStarted() && !isStopRequested())");
            dashboardTelemetry.addData("minT", dashboardTelemetry.getMsTransmissionInterval());
            dashboardTelemetry.addData("Error1", 0.0);
            dashboardTelemetry.addData("Error2", 0.0);
            dashboardTelemetry.addData("Error3", 0.0);
            dashboardTelemetry.update();
            sleep(20);
        }

//        drive.followTrajectorySequenceAsync(path);
        lastTime = timer.milliseconds();

        while (!isStopRequested() && opModeIsActive()) {
            dashboardTelemetry.addData("timer", timer.milliseconds());
            switch (mState) {
                case drivePose:
                    dashboardTelemetry.addLine("drive pose updating");
                    dashboardTelemetry.addData("AprilTagX", myCamera.getAprilTagIDData(10)[2]);
                    drive.update();
                    if (!isAutoEnd) {
                        drive.followTrajectorySequenceAsync(path);
                        isAutoEnd = true;
                    }
                    break;
                case alignAprilTag:
                    dashboardTelemetry.addLine("align aprilTag updating");
                    double[] idData = myCamera.getAprilTagIDData(10);
                    drive.alignAprilTag(16.0, 0.0, 0.0, idData[0], idData[1], idData[2], idData[3]);
                    if (drive.isEndAlign()) {
//                        lastPose = drive.getPoseEstimate();
//                        drive.setPoseEstimate(lastPose);
                        mState = Auto_State.PARK;
                    }
                    break;
                case PARK:
                    dashboardTelemetry.addLine("drive park");
                    drive.followTrajectorySequenceAsync(path2);
                    mState = Auto_State.STOP;
                    break;
                case STOP:
                    dashboardTelemetry.addLine("STOP Now");
                    drive.update();
                    break;
            }
//            drive.update();
////            turret.loop();
//            testMotor.loop();
//            currentTime = timer.milliseconds();
//            dashboardTelemetry.addLine("run-time: " + currentTime/1000);
//            dashboardTelemetry.addLine("loop-time: " + (currentTime - lastTime)/1000);
//            if (currentTime > 7000.0 && !isAutoEnd) {
//                drive.followTrajectorySequenceAsync(leftPark);
//                isAutoEnd = true;
//            }
//            lastTime = currentTime;
        }
    }
}
