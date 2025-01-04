package org.firstinspires.ftc.teamcode.AutoTest.Roadrunner;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.AutoTest.Roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.AutoTest.Roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.TeleOps.RobotActionConfig;
import org.firstinspires.ftc.teamcode.TeleOps.RobotHardware;

public class RightSideAuto extends LinearOpMode {

    public static double basket_x_coordinate = -18;
    public static double basket_y_coordinate = -7;
    public static double first_sample_x_coordinate = -7;
    public static double first_sample_y_coordinate = -35;
    public static double second_sample_x_coordinate = -18;
    public static double second_sample_y_coordinate = -35;
    public static double third_sample_x_coordinate = -18;
    public static double third_sample_y_coordinate = -35;
    public static double third_sample_heading = -120;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        RobotActionConfig value = new RobotActionConfig();
        RobotHardware robot = new RobotHardware();
        robot.init(hardwareMap);

        Pose2d startPose = new Pose2d(0, 0, 0);

        drive.setPoseEstimate(startPose);

        TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(basket_x_coordinate, basket_y_coordinate,Math.toRadians(-45)))
                //.addDisplacementMarker(() -> {})
                .waitSeconds(6)
                .lineToLinearHeading(new Pose2d(first_sample_x_coordinate, first_sample_y_coordinate,Math.toRadians(-90)))
                //.addDisplacementMarker(() -> {})
                .waitSeconds(3)
                .lineToLinearHeading(new Pose2d(basket_x_coordinate, basket_y_coordinate,Math.toRadians(-45)))
                //.addDisplacementMarker(() -> {})
                .waitSeconds(6)
                .lineToLinearHeading(new Pose2d(second_sample_x_coordinate, second_sample_y_coordinate,Math.toRadians(-90)))
                //.addDisplacementMarker(() -> {})
                .waitSeconds(3)
                .lineToLinearHeading(new Pose2d(basket_x_coordinate, basket_y_coordinate,Math.toRadians(-45)))
                //.addDisplacementMarker(() -> {})
                .waitSeconds(6)
                .lineToLinearHeading(new Pose2d(third_sample_x_coordinate, third_sample_y_coordinate,Math.toRadians(third_sample_heading)))
                //.addDisplacementMarker(() -> {})
                .waitSeconds(3)
                .build();

        waitForStart();

        if (!isStopRequested())
            drive.followTrajectorySequence(trajSeq);
    }
}
