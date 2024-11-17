package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

@Autonomous(name = "RoadRunner Autonomous", group = "Autonomous")
public class RoadRunnerAuto extends AutoBase {
    private MecanumDrive drive;

    @Override
    public void runOpMode() {
        super.runOpMode(); // Run menu selection
        
        // Initialize drive system
        drive = new MecanumDrive(hardwareMap, getStartingPose());
        
        waitForStart();
        if (isStopRequested()) return;
        
        runAuto();
    }

    private void runAuto() {
        if (Settings.Deploy.SKIP_AUTONOMOUS) {
            justPark();
            return;
        }
        
        // TODO: Add actual autonomous path here
    }

    private void justPark() {
        double strafeDistance = startingPosition.equals("Right") ? -36 : 36;
        drive.actionBuilder(getStartingPose())
                .splineTo(
                    new Vector2d(
                        getStartingPose().position.x + strafeDistance, 
                        getStartingPose().position.y
                    ),
                    getStartingPose().heading
                )
                .build();
    }

    private Pose2d getStartingPose() {
        double x = startingPosition.equals("Right") ? 36 : -36;
        double y = color.equals("Red") ? -60 : 60;
        double heading = color.equals("Red") ? Math.toRadians(90) : Math.toRadians(-90);
        
        return new Pose2d(x, y, heading);
    }
}