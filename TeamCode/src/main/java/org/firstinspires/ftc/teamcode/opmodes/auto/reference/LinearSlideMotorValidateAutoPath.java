package org.firstinspires.ftc.teamcode.opmodes.auto.reference;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.opmodes.auto.presets.Arm;
import org.firstinspires.ftc.teamcode.opmodes.auto.presets.LinearSlide;
import org.firstinspires.ftc.teamcode.opmodes.auto.presets.XYaw;
import org.firstinspires.ftc.teamcode.opmodes.auto.presets.YPitch;
import org.firstinspires.ftc.teamcode.roadrunner.teamcode.MecanumDrive;

@Autonomous(name = "Linear Slide Motor Validate", group = "Initial Validation")
public class LinearSlideMotorValidateAutoPath extends LinearOpMode {

    @Override
    public void runOpMode() {
        // Initialize the MecanumDrive with the hardware map
        Pose2d initialPose = new Pose2d(-24, 60, Math.toRadians(-90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        Arm arm = new Arm(hardwareMap);
        LinearSlide linearSlide = new LinearSlide(hardwareMap);
        YPitch pitch = new YPitch(hardwareMap);
        XYaw yaw = new XYaw(hardwareMap);

        TrajectoryActionBuilder waitTrajectory = drive.actionBuilder(initialPose)
                .waitSeconds(1);

        if (isStopRequested()) return;

        while (!isStopRequested() && !opModeIsActive()) {

            telemetry.addData("Position during Init", drive.updatePoseEstimate());
            telemetry.update();
        }


        // Wait for the start signal
        waitForStart();

        if (opModeIsActive()) {
            // Define the trajectory for the Blue Basket sequence with waits
            Actions.runBlocking(
                    new SequentialAction(
//                            arm.initializeArm(),
//                            waitTrajectory.build(),
                            arm.raiseArmForLowerBasket(),
//                            arm.raiseArmForLowerBasket(),
//                            linearSlide.initLinearSlide(),
                            linearSlide.extendArmForward(),
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
                            new SleepAction(1),
                            linearSlide.retractSlideBackward(),
//                            linearSlide.extendSlideForPickFromPool(),
                            new SleepAction(1),
                            arm.deactivateArm()
                    )
            );
        }
    }
}