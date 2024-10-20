package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name = "Spline-test")
public class SplineAutonomous extends LinearOpMode {
    public Robot robot;
    public SampleMecanumDrive drive;
    public Slider slider;
    public Arm arm;
    public Claw claw;
    public AprilTag tag;
    public TgeDetection tge;
    String curAlliance = "red";
    public int zone = 2;
    TrajectorySequence trajBlueZone1;
    Leds leds;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap, telemetry);
        drive = robot.sampleDrive;
        slider = new Slider(robot, gamepad2);
        arm = new Arm(robot, gamepad2);
        claw = new Claw(robot.servoCR, robot.claw_left_close,robot.claw_left_wide_close, robot.claw_left_open, robot.servoCL, robot.claw_right_close,  robot.claw_right_wide_close, robot.claw_right_open);
        leds = new Leds(robot);
        arm.setPosFold();

        //tag = new AprilTag(robot);
        tge = new TgeDetection(robot, "red");

        buildBlueZone1Trajectory();

        waitForStart();
        if(isStopRequested()) {
            return;
        }
        if(opModeIsActive()) {
            zone = 1;
            switch(zone) {
                case 1:
                    Pose2d startPose = new Pose2d(-35.5, 64, Math.toRadians(270));
                    drive.setPoseEstimate(startPose);
                    robot.sampleDrive.followTrajectorySequence(trajBlueZone1);;
                    break;
            }
        }

    }

    private void buildBlueZone1Trajectory() {
        Pose2d startPose = new Pose2d(-35.5, 64, Math.toRadians(270));
        drive.setPoseEstimate(startPose);
        trajBlueZone1 = drive.trajectorySequenceBuilder(startPose)
                .waitSeconds(1)
                //.setVelConstraint(SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH) )
                //.setAccelConstraint( SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                //.setTurnConstraint(18, 20)
                .addTemporalMarker(() -> {
                    claw.close();
                    sleep(500);
                })
                .waitSeconds(0.5)
                .splineTo(new Vector2d(-35.5, 11),Math.toRadians(270))
                .waitSeconds(0.5)
                .turn(Math.toRadians(180))
                .forward(1)
                .addTemporalMarker(() -> {
                    arm.setPosSample();
                    sleep(500);
                    claw.open();
                    sleep(500);
                    arm.setPosSpecimen();
                    sleep(500);
                    claw.close();
                    sleep(500);
                })
                .back(1)
                .waitSeconds(0.5)
                .turn(Math.toRadians(-90))
                .waitSeconds(0.5)
                .splineTo(new Vector2d(20, 11),Math.toRadians(0))
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH) )
                .splineTo(new Vector2d(52, 38),Math.toRadians(0))
                .addTemporalMarker(() -> {
                    slider.auton();
                    sleep(1000);
                    claw.open();
                    arm.setPosFold();
                })

                .back(3)
                .waitSeconds(0.5)
                //.resetVelConstraint()
                //.splineTo(new Vector2d(40, 34),Math.toRadians(0))
                .back(8)
                .waitSeconds(0.5)
                .turn(Math.toRadians(-90))
                .forward(23)
                .waitSeconds(0.5)
                .turn(Math.toRadians(-90))
                .back(20)
                //.splineTo(new Vector2d(40, 11),Math.toRadians(0))
                //.splineTo(new Vector2d(60, 19),Math.toRadians(0))
                .build();
    }
}