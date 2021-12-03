package org.firstinspires.ftc.teamcode.src.DrivePrograms.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.src.DrivePrograms.Misc.AutoObjDetectionTemplate;
import org.firstinspires.ftc.teamcode.src.Utills.MiscUtills;
import org.firstinspires.ftc.teamcode.src.robotAttachments.Subsystems.LinearSlide;


@Autonomous(name = "BlueWarehouseAutonomous")
public class BlueWarehouseAutonomous extends AutoObjDetectionTemplate {

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


        odometry.setPosition(133, 63, 270);
        waitForStart();
        tfod.shutdown();
        vuforia.close();
        System.gc();


        switch (Pos) {
            case NotSeen:
                telemetry.addData("position", " is far left");
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
                driveSystem.turnTo(190, .5);
                //Thread.sleep(500);
                driveSystem.moveToPosition(131, 63, 2);

                driveSystem.moveToPosition(132, 24, 1);


                break;
            case Right:
                telemetry.addData("position", " is right");
                telemetry.update();
                driveSystem.moveToPosition(27, 85, 1);

                slide.setTargetLevel(LinearSlide.HeightLevels.BottomLevel);


                break;
            case Left:
                telemetry.addData("position", "is center");
                telemetry.update();
                driveSystem.moveToPosition(27, 85, 1);

                slide.setTargetLevel(LinearSlide.HeightLevels.MiddleLevel);


                break;


        }
        telemetry.update();
        while (opModeIsActive() && !isStopRequested()) ;

    }
}


