package org.firstinspires.ftc.teamcode.opmodes.auto.red;


// RR-specific imports

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
import org.firstinspires.ftc.teamcode.opmodes.auto.presets.Roll;
import org.firstinspires.ftc.teamcode.opmodes.auto.presets.XYaw;
import org.firstinspires.ftc.teamcode.opmodes.auto.presets.YPitch;
import org.firstinspires.ftc.teamcode.roadrunner.teamcode.MecanumDrive;

@Autonomous(name="Red Sample Drop", group = "Red Alliance", preselectTeleOp = "RobotCentricDrive")

public class RedSampleDrop extends LinearOpMode {
    public void runOpMode() {
        // I'm assuming you're at 0,0
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(12, -60,0 ));
        // Define arm positions using the constants from the Arm class

//        Arm arm = new Arm(hardwareMap);
//
//        XYaw yaw = new XYaw(hardwareMap);
//        YPitch pitch=new YPitch(hardwareMap);
        // Define arm positions using the constants from the Arm class

        Arm arm = new Arm(hardwareMap);
        LinearSlide linearSlide = new LinearSlide(hardwareMap);

        XYaw yaw = new XYaw(hardwareMap);
        YPitch pitch = new YPitch(hardwareMap);
        Roll roll = new Roll(hardwareMap);
        Claw claw = new Claw(hardwareMap);

        waitForStart();

        Actions.runBlocking(
                new SequentialAction(
                        drive.actionBuilder(new Pose2d(12, -60,Math.toRadians(0)))
//                                .afterDisp(.5,  arm.raiseArmForLowerBasket())
//                                .waitSeconds(.100)
//                                .afterDisp(.5, yaw.moveWristCenter())
                                .strafeTo(new Vector2d(0,-33))
                                .waitSeconds(.100)
//                        arm.initializeArm(),
//                        arm.raiseArmForSpecimenPickUpFromWall(),
//                        drive.actionBuilder(new Pose2d(0, -36, 0))
                                .lineToX(35)
                                //.splineTo(new Vector2d(35, 0), Math.toRadians(90))
                                .turnTo(Math.toRadians(90))
                                .lineToY(-2)
                                .waitSeconds(.100)
                                .strafeTo(new Vector2d(48, -6))
                                .waitSeconds(.100)
                                .lineToY(-48)
                                .lineToY(-2)
                                .strafeTo(new Vector2d(57, -6))
                                .waitSeconds(.100)
                                .lineToY(-48)
                                .lineToY(-2)
                                .strafeTo(new Vector2d(62, -6))
                                .waitSeconds(.100)
                                .lineToY(-2)
                                .waitSeconds(.100)
                                .lineToY(-15)
                                .turnTo(Math.toRadians(270))
                                .lineToY(-6)
                                .build()));




    }
}