package org.firstinspires.ftc.teamcode.opmodes.test.auto;

// RR-specific imports
import com.acmerobotics.roadrunner.Pose2d;
        import com.acmerobotics.roadrunner.ftc.Actions;

// Non-RR imports
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

        import org.firstinspires.ftc.teamcode.roadrunner.teamcode.MecanumDrive;

@Autonomous(name = "Roadrunner Basic Auto")
@Disabled
public class RoadRunner_Test extends LinearOpMode {

    @Override
    public void runOpMode() {
        // I'm assuming you're at 0,0
        Pose2d initPose = new Pose2d(0, 61.75, Math.toRadians(90.00));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initPose);

        waitForStart();

        Actions.runBlocking(
                drive.actionBuilder(initPose)
                        .lineToY(45)
                        .lineToY(29.66)
                        .lineToY(0)
                        .lineToXSplineHeading(23,Math.toRadians(180.00))
                        .build());
    }

}