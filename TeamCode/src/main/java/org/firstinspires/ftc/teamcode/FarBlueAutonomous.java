package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name = "Far-Blue-Autonomous")
public class FarBlueAutonomous extends LinearOpMode {
    public Robot robot;
    public SampleMecanumDrive drive;
    public Slider slider;
    public Arm arm;
    public Claw claw;
    public AprilTag tag;
    public TgeDetection tge;
    String curAlliance = "red";
    public int zone = -1;
    TrajectorySequence trajBlueZone1;
    TrajectorySequence trajBlueZone2;
    TrajectorySequence trajBlueZone3;
    Leds leds;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap, telemetry);
        drive = robot.sampleDrive;
        slider = new Slider(robot, gamepad2);
        arm = new Arm(robot, gamepad2);
        claw = new Claw(robot.servoCR, robot.claw_left_close,robot.claw_left_wide_close, robot.claw_left_open, robot.servoCL, robot.claw_right_close,  robot.claw_right_wide_close, robot.claw_right_open);
        leds = new Leds(robot);
        leds.setPattern(0);

        // Fold and open the claws for placing the pixels
        arm.setPosFold();
        claw.close();
        sleep(500);
        claw.open();
        sleep(500);

        //tag = new AprilTag(robot);
        tge = new TgeDetection(robot, "blue");
        buildBlueZone1Trajectory();
        buildBlueZone2Trajectory();
        buildBlueZone3Trajectory();

        // Get the zone a few times to avoid random values
        while(tge.getZone() == -1) {
            telemetry.addData("CAMERA INIT:", tge.getZone());
            telemetry.update();
            sleep(100);
        }
        zone = tge.getZone();
        telemetry.addData("INIT Zone number:", zone);
        telemetry.update();

        // Wait for start button to be pressed
        waitForStart();
        if(isStopRequested()) {
            return;
        }
        //Now the Auton Started
        claw.close();

        zone = tge.getZone();
        telemetry.addData("Zone number:", zone);
        telemetry.update();
        tge.stop();
        sleep(4000);
        if(opModeIsActive()) {
            //zone = 2;
            switch(zone) {
                case 1:
                    // Zone1
                    robot.sampleDrive.followTrajectorySequence(trajBlueZone1);;
                    break;
                case 2:
                    // Zone2
                    robot.sampleDrive.followTrajectorySequence(trajBlueZone2);;
                    break;
                case 3:
                    // Zone3
                    robot.sampleDrive.followTrajectorySequence(trajBlueZone3);;
                    break;
            }
        }
        slider.fold();
        claw.close();
        claw.close();
        leds.setPattern(10);
        sleep(1000);
    }

    // Build Zone1 trajectory
    private void buildBlueZone1Trajectory() {
        Pose2d startPose = new Pose2d(0, 0, 0);
        drive.setPoseEstimate(startPose);
        trajBlueZone1 = drive.trajectorySequenceBuilder(startPose)
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    arm.setPosSample();
                    sleep(500);
                })
                .waitSeconds(0.5)
                .forward(22)
                .turn(Math.toRadians(50))
                .forward(4)     // Now at the zone1
                // Drop the purple pixel
                .addTemporalMarker(() -> {
                    claw.open();
                    sleep(500);
                    arm.setPosSpecimen();  //places purple pixel
                    sleep(500);
                })
                // Now go near the gate and turn toward the backdrop
                .back(3)
                .turn(Math.toRadians(-50))
                .forward(30)
                .turn(Math.toRadians(90))

                // Go through middle fence
                .forward(72)
                .strafeLeft(35.5)
                .addTemporalMarker(() -> {
                    slider.auton();
                    sleep(1000);
                })
                .waitSeconds(0.5)
                .forward(14.235)
                .waitSeconds(0.5)
                // Drop the yellow pixel
                .addTemporalMarker(() -> {
                    claw.open();
                    sleep(500);
                    arm.setPosFold(); //places pixel on the backdrop
                    sleep(500);
                })
                .waitSeconds(1)
                // Go to the parking in the center side
                .back(15)
                .strafeRight(32)
                .turn(Math.toRadians(180))
                .back(23) // Now at the parking
                .build();

    }

    // Zone2 trajectory
    private void buildBlueZone2Trajectory() {
        //Pose2d startPose = new Pose2d(-35.5, 64, Math.toRadians(270));
        Pose2d startPose = new Pose2d(0, 0, 0);
        //drive.setPoseEstimate(startPose);
        trajBlueZone2 = drive.trajectorySequenceBuilder(startPose)
                // Go to the zone2
                .forward(50.5)
                .turn(Math.toRadians(185))  // Now at the zone2
                .waitSeconds(0.5)
                // Drop the purple pixel
                .addTemporalMarker(() -> {
                    arm.setPosSample();
                    sleep(500);
                    claw.open();
                    sleep(500);
                    arm.setPosSpecimen(); //places purple pixel
                    sleep(500);
                })
                // Position near the gate
                .back(6.5)
                .turn(Math.toRadians(-93)) // face the backdrop
                .waitSeconds(0.5)

                // Go to the backdrop through the gate
                .forward(70)
                .strafeLeft(28.8)
                .addTemporalMarker(() -> {
                    slider.auton();
                    sleep(1000);
                })
                .waitSeconds(0.5)
                .forward(16.15) //now at the backdrop
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    claw.open();
                    claw.open();
                    sleep(500);
                    arm.setPosFold(); //places pixel on the back drop
                    sleep(500);
                })
                // Go to the parking at the center
                .back(10)
                .waitSeconds(0.5)
                .strafeRight(21.5)
                .waitSeconds(0.5)
                .turn(Math.toRadians(180))
                .waitSeconds(0.5)
                .back(15) //now parked
                .build();
    }

    // Zone3 trajectory
    private void buildBlueZone3Trajectory() {
        Pose2d startPose = new Pose2d(0, 0, 0);
        drive.setPoseEstimate(startPose);
        trajBlueZone3 = drive.trajectorySequenceBuilder(startPose)
                .waitSeconds(0.5)
                // Drop the arm
                .addTemporalMarker(() -> {
                    arm.setPosSample();
                    sleep(500);
                })
                .waitSeconds(0.5)
                .forward(23)
                .turn(Math.toRadians(-50))
                .forward(1) // Now at the zone3
                // Drop the purple pixel
                .addTemporalMarker(() -> {
                    claw.open();
                    sleep(500);
                    arm.setPosSpecimen();
                    sleep(500);
                })
                .back(2)
                .turn(Math.toRadians(50))
                .forward(32.5)
                .turn(Math.toRadians(90)) // Face the backdrop now
                .waitSeconds(0.5)

                // Go through the gate
                .forward(72)
                .strafeLeft(25.5)
                .addTemporalMarker(() -> {
                    slider.auton();
                    sleep(1000);
                })
                .waitSeconds(0.5)
                .forward(14.56)
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    claw.open(); //places pixel on the backdrop
                    sleep(500);
                    arm.setPosFold();
                    sleep(500);
                })
                .waitSeconds(0.5)
                // Go to the parking
                .back(15)
                .strafeRight(17)
                .turn(Math.toRadians(180))
                .back(23) // Parked at the center
                .build();
    }

}

