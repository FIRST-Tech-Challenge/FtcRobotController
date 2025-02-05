package org.firstinspires.ftc.teamcode.opmodes.auto.blue;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.roadrunner.teamcode.MecanumDrive;

@Autonomous(name = "BluePushToDestPath", group = "Blue Alliance", preselectTeleOp = "RobotCentricDrive")
@Disabled
public class BluePushToDestPath extends LinearOpMode {
    @Override
    public void runOpMode() {
        // Initialize the MecanumDrive with the hardware map
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(-24, 60, Math.toRadians(-90)));

       // Arm arm = new Arm(hardwareMap);
        //LinearSlide linearSlide = new LinearSlide(hardwareMap);
        //Claw claw = new Claw(hardwareMap);
        //XYaw yaw = new XYaw(hardwareMap);
        //YPitch pitch = new YPitch(hardwareMap);

       // if (isStopRequested()) return;

        // Wait for the start signal
        waitForStart();

        if (opModeIsActive()) {
            // Define the trajectory for the Blue Basket sequence with waits
            Actions.runBlocking(
                    drive.actionBuilder(new Pose2d(-24, 60, Math.toRadians(-90)))
//                .strafeTo(new Vector2d(-36,12))
                            .waitSeconds(1)
                            .splineTo(new Vector2d(-40, 12), -90)
                            .waitSeconds(1)
//                .lineToY(20)
                            .turnTo(Math.toRadians(-180))
                            .waitSeconds(1)
                            .lineToX(-46)
                            .waitSeconds(1)
                            .turnTo(Math.toRadians(-180))
                            .waitSeconds(1)
                            .strafeTo(new Vector2d(-48,50))
                            .waitSeconds(1)
                            .strafeTo(new Vector2d(-46, 12))
                            .waitSeconds(1)
                            .lineToX(-54)
                            .waitSeconds(1)
                            .strafeTo(new Vector2d(-56,50))
                            .waitSeconds(1)
                            .lineToX(-54)
                            .waitSeconds(1)
                            .turnTo(Math.toRadians(360))
                            .waitSeconds(1)
                            .strafeTo(new Vector2d(-54,12))
                            .waitSeconds(1)
                            .lineToX(-62)
                            .waitSeconds(1)
                            .strafeTo(new Vector2d(-62,48))
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