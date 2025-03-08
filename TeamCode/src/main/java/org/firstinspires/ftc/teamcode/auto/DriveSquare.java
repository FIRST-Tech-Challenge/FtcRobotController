package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@Autonomous(name = "Square Auto", group = "BlueViii-auto")
public class DriveSquare extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        Servo rotateArm    = hardwareMap.get(Servo.class, "rotateArm");

        telemetry.setMsTransmissionInterval(50);

        Pose2d initPos = new Pose2d(0, 0, 0);
        MecanumDrive drive = new MecanumDrive(hardwareMap, initPos);

        while (!isStarted() && !isStopRequested()) {
            telemetry.update();
        }

        waitForStart();

        while (opModeIsActive()) {
            TrajectoryActionBuilder trajBuilder = drive.actionBuilder(initPos)
                    .lineToX(12)
                    .turn(90)
                    .lineToY(12);

            Actions.runBlocking(trajBuilder.build());
        }
    }
}
