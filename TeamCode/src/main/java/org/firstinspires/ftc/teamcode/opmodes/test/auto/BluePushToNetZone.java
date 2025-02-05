package org.firstinspires.ftc.teamcode.opmodes.test.auto;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.opmodes.auto.presets.Arm;
import org.firstinspires.ftc.teamcode.opmodes.auto.presets.Claw;
import org.firstinspires.ftc.teamcode.opmodes.auto.presets.LinearSlide;
import org.firstinspires.ftc.teamcode.opmodes.auto.presets.XYaw;
import org.firstinspires.ftc.teamcode.opmodes.auto.presets.YPitch;
import org.firstinspires.ftc.teamcode.roadrunner.teamcode.MecanumDrive;

@Autonomous(name = "BluePushToNetPath")
@Disabled
public class BluePushToNetZone extends LinearOpMode {
    @Override
    public void runOpMode() {
        // Initialize the MecanumDrive with the hardware map
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(-24, 60, Math.toRadians(-90)));

        Arm arm = new Arm(hardwareMap);
        LinearSlide linearSlide = new LinearSlide(hardwareMap);
        Claw claw = new Claw(hardwareMap);
        XYaw yaw = new XYaw(hardwareMap);
        YPitch pitch = new YPitch(hardwareMap);

        if (isStopRequested()) return;

        waitForStart();

        if (opModeIsActive()) {
            // Define the trajectory for the Blue Basket sequence with waits
            Actions.runBlocking(
                    new SequentialAction(drive.actionBuilder(new Pose2d(24, 60, Math.toRadians(-90)))
                            .waitSeconds(1)
                            .splineToLinearHeading(new Pose2d(46, 12, Math.toRadians(-90)), 0)
                            .waitSeconds(1)
                            .strafeTo(new Vector2d(48, 48))
                            .turnTo(Math.toRadians(-135))
                            .waitSeconds(1)
                            .strafeTo(new Vector2d(54, 54))
                            .waitSeconds(1)
                            .splineToLinearHeading(new Pose2d(54, 12, Math.toRadians(-90)), 0)
                            .waitSeconds(1)
                            .strafeTo(new Vector2d(54, 54))
                            .turnTo(Math.toRadians(-135))
                            .waitSeconds(1)
                            .splineToLinearHeading(new Pose2d(60, 12, Math.toRadians(-90)), 0)
                            .strafeTo(new Vector2d(60, 54))
                            .build()
            ));
        }
    }
}