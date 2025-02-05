package org.firstinspires.ftc.teamcode.opmodes.auto.OpenCV;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.roadrunner.teamcode.MecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@Autonomous(name = "Test Vision", group = "Experiment")
@Disabled
public class TestOpenCVPath extends LinearOpMode {

    private AprilTagProcessor aprilTagProcessor;

    private VisionPortal visionPortal;



    @Override
    public void runOpMode() {
        // Initialize the MecanumDrive with the hardware map
        Pose2d initialPose = new Pose2d(-24, 60, Math.toRadians(-90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);


        TrajectoryActionBuilder waitTrajectory = drive.actionBuilder(initialPose)
                .waitSeconds(1);

        if (isStopRequested()) return;

        while (!isStopRequested() && !opModeIsActive()) {

            telemetry.addData("Position during Init", drive.updatePoseEstimate());
            telemetry.update();
        }

        Pose2d initPose = new Pose2d(-24, 60, Math.toRadians(-90));

        TrajectoryActionBuilder testTrajectory = drive.actionBuilder(initPose);

        // Wait for the start signal
        waitForStart();

        if (opModeIsActive()) {



            // Define the trajectory for the Blue Basket sequence with waits
            Actions.runBlocking(
                    new SequentialAction(testTrajectory.build()

//                            new ParallelAction(testTrajectory.build()),
//                            arm.initializeArm(),
//                            waitTrajectory.build(),
//                            arm.raiseArmForLowerBasket(),
//                            linearSlide.initLinearSlide(),
//                            linearSlide.extendArmForward(),
//                            arm.raiseArmForNetzone(),
//                            waitTrajectory.build(),
//                            arm.raiseArmForSamplePickUpFromFloor(),
//                            waitTrajectory.build(),
//                            arm.raiseArmForLowerBasket(),
//                            arm.raiseArmForSamplePickUpFromFloor(),
//                            waitTrajectory.build(),
//                            arm.raiseArmForUpperBasket(),
//                            waitTrajectory.build(),
//                            arm.initializeArm(),
//                            linearSlide.retractArmBackward(),
//                            linearSlide.extendSlideForPickFromPool(),

                    )
            );
        }
    }
}