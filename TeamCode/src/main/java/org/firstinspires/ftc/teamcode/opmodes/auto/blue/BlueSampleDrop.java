package org.firstinspires.ftc.teamcode.opmodes.auto.blue;


// RR-specific imports

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.opmodes.auto.presets.Arm;
import org.firstinspires.ftc.teamcode.opmodes.auto.presets.XYaw;
import org.firstinspires.ftc.teamcode.opmodes.auto.presets.YPitch;
import org.firstinspires.ftc.teamcode.roadrunner.teamcode.MecanumDrive;

@Autonomous(name="Blue Sample Drop", group = "Blue Alliance", preselectTeleOp = "RobotCentricDrive")
@Disabled
public class BlueSampleDrop extends LinearOpMode {
    public void runOpMode() {
        // I'm assuming you're at 0,0
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 60,0 ));
        // Define arm positions using the constants from the Arm class

        Arm arm = new Arm(hardwareMap);

        XYaw yaw = new XYaw(hardwareMap);
        YPitch pitch=new YPitch(hardwareMap);
        waitForStart();

        Actions.runBlocking(
                new SequentialAction(
                        drive.actionBuilder(new Pose2d(0, 60,Math.toRadians(0)))
                                .afterDisp(.5,  arm.raiseArmForLowerBasket())
                                .waitSeconds(.100)
                                .afterDisp(.5, yaw.moveWristCenter())
//                        .waitSeconds(.100)
                                .strafeTo(new Vector2d(0, 33))
                        //.turnTo(Math.toRadians(90))
                        //.lineToY(-36)
                        .waitSeconds(.100)
//                        .build()
//                        ));
//                        arm.initializeArm(),
//                        arm.raiseArmForSpecimenPickUpFromWall(),
//                        drive.actionBuilder(new Pose2d(0, -36, 0))
                        //.turnTo(Math.toRadians(0))
                        .lineToX(-36)
                        .turnTo(Math.toRadians(-90))
                        .lineToY(6)
                        .waitSeconds(.100)
                        .strafeTo(new Vector2d(-48, 6))
                        .waitSeconds(.100)
                        .lineToY(58)
                        .lineToY(6)
                        .strafeTo(new Vector2d(-56, 6))
                        .waitSeconds(.100)
                        .lineToY(58)
                        .lineToY(6)
                        .strafeTo(new Vector2d(-60, 6))
                        .waitSeconds(.100)
                        .lineToY(58)
                       // .strafeTo(new Vector2d(57, -52))
                        //.waitSeconds(.100)
                        //.turnTo(Math.toRadians(270))
//                        build()
                       // arm.initializeArm(),
                        //arm.raiseArmForSpecimenPickUpFromWall(),
//                        drive.actionBuilder(new Pose2d(54, -52, 0))
                        //.turnTo(Math.toRadians(90))
                        //.lineToY(-40)
                        //.waitSeconds(.100)
                        //.turnTo(Math.toRadians(180))
                        //.lineToX(2)
                        //.turnTo(Math.toRadians(90))
                                .build()));




    }
}
