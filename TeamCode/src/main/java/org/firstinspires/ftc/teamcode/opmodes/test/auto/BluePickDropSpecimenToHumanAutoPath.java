package org.firstinspires.ftc.teamcode.opmodes.test.auto;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.opmodes.auto.presets.Arm;
import org.firstinspires.ftc.teamcode.opmodes.auto.presets.LinearSlide;
import org.firstinspires.ftc.teamcode.roadrunner.teamcode.MecanumDrive;

@Autonomous(name = "BluePickDropSpecimenToHumanAutoPath")
@Disabled
public class BluePickDropSpecimenToHumanAutoPath extends LinearOpMode {
    @Override
    public void runOpMode() {
        // Initialize the MecanumDrive with the hardware map
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(-24, 60, Math.toRadians(-90)));
        Arm arm = new Arm(hardwareMap);
        LinearSlide linearSlide = new LinearSlide(hardwareMap);

        // Wait for the start signal
        waitForStart();

        if (isStopRequested()) return;

        while (!isStopRequested() && !opModeIsActive()) {
          // Define the trajectory for the Blue Basket sequence with waits
            Actions.runBlocking(
                    drive.actionBuilder(new Pose2d(-24, 60, Math.toRadians(-90)))
                            .waitSeconds(1)
                            .splineToLinearHeading(new Pose2d(-36, 24, Math.toRadians(-180)), -90)
                            .waitSeconds(1)
                            .strafeTo(new Vector2d(-36, 48))
                            .turnTo(Math.toRadians(135))
                            .waitSeconds(1)
                            .strafeTo(new Vector2d(-44, 24))
                            .turnTo(Math.toRadians(180))
                            .waitSeconds(1)
                            .strafeTo(new Vector2d(-44, 48))
                            .turnTo(Math.toRadians(135))
                            .strafeTo(new Vector2d(-53, 24))
                            .turnTo(Math.toRadians(180))
                            .waitSeconds(1)
                            .strafeTo(new Vector2d(-53, 48))
                            .turnTo(Math.toRadians(90))
                            .waitSeconds(1)
                            .build()


//                    new Pose2d(-24, 58, Math.toRadians(-90)))
//                            .strafeTo(new Vector2d(-36,10))
//                            .strafeTo(new Vector2d(-48,57))
////                            .strafeTo(new Vector2d(-47,10))
////                            .strafeTo(new Vector2d(-60,57))
////                            .strafeTo(new Vector2d(-55,10))
////                            .strafeTo(new Vector2d(-61,57))
//                            .build()
            );
        }
    }
}