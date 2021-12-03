package org.firstinspires.ftc.teamcode.src.DrivePrograms.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.src.DrivePrograms.Misc.AutoObjDetectionTemplate;
import org.firstinspires.ftc.teamcode.src.Utills.MiscUtills;
import org.firstinspires.ftc.teamcode.src.robotAttachments.Subsystems.LinearSlide;


@Autonomous(name = "BlueCarouselAuto")
public class BlueCarouselAuto extends AutoObjDetectionTemplate {

    @Override
    public void runOpMode() throws InterruptedException {

        this.initAll();
        this.initVuforia();
        this.initTfod();

        this.activate();
        MarkerPosition Pos = MarkerPosition.Left;

        try {
            //Pos = this.getAverageOfMarker(10, 100);

            Pos = this.findPositionOfMarker();
        } catch (Exception e) {

            telemetry.addData("Error", MiscUtills.getStackTraceAsString(e));
            telemetry.update();
            while (opModeIsActive() && !isStopRequested()) {
            }

        }


        odometry.setPosition(7, 63, 90);
        waitForStart();
        tfod.shutdown();
        vuforia.close();
        System.gc();


        switch (Pos) {
            case NotSeen:
                telemetry.addData("position", " is far right");
                telemetry.update();

                driveSystem.moveToPosition(120, 84, 1);

                slide.setTargetLevel(LinearSlide.HeightLevels.TopLevel);
                intake.setServoDown();

                Thread.sleep(500);
                driveSystem.moveToPosition(111, 84, 1);
                Thread.sleep(500);
                intake.intakeOn();
                Thread.sleep(1000);
                intake.intakeOff();
                driveSystem.moveToPosition(120, 84, 1);
                intake.setServoUp();

                driveSystem.moveToPosition(126, 112, 1, true);
                driveSystem.turnTo(180, .5);
                driveSystem.moveToPosition(126, 132, 1.5, true);
                driveSystem.strafeAtAngle(180 + 35, .5);
                Thread.sleep(500);
                spinner.spinOffBlueDuck();
                driveSystem.moveToPosition(105, 132, 1);
                driveSystem.strafeAtAngle(180, 0.5);
                Thread.sleep(1000);


                break;
            case Right:
                telemetry.addData("position", " is center");
                telemetry.update();
                driveSystem.moveToPosition(27, 85, 1);

                slide.setTargetLevel(LinearSlide.HeightLevels.MiddleLevel);


                break;
            case Left:
                telemetry.addData("position", "is left");
                telemetry.update();
                driveSystem.moveToPosition(27, 85, 1);

                slide.setTargetLevel(LinearSlide.HeightLevels.BottomLevel);


                break;


        }
        telemetry.update();
        while (opModeIsActive() && !isStopRequested()) ;

    }
}


