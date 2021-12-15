package org.firstinspires.ftc.teamcode.src.DrivePrograms.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.src.Utills.AutoObjDetectionTemplate;
import org.firstinspires.ftc.teamcode.src.robotAttachments.Subsystems.LinearSlide;


@Autonomous(name = "BlueWarehouseAutonomous")
public class BlueWarehouseAutonomous extends AutoObjDetectionTemplate {

    @Override
    public void runOpMode() throws InterruptedException {
        this.initAll();
        MarkerPosition Pos = MarkerPosition.NotSeen;
        slide.setTargetLevel(LinearSlide.HeightLevels.Down);
        this.initVuforia();
        this.initTfod();
        this.activate();
        odometry.setPosition(133, 63, 270);

        while (!isStarted() && !isStopRequested()) {
            Pos = this.getAverageOfMarker(10, 100);
            telemetry.addData("Position", Pos);
            telemetry.update();
        }

        waitForStart();

        if (opModeIsActive() && !isStopRequested()) {
            tfod.shutdown();
            vuforia.close();
            System.gc();


            switch (Pos) {
                case Right:
                    telemetry.addData("position", " is far right");
                    telemetry.update();

                    driveSystem.moveToPosition(120, 84, 1);

                    slide.setTargetLevel(LinearSlide.HeightLevels.TopLevel);
                    Thread.sleep(1500);


                    //this position will vary for different heights on the goal
                    driveSystem.moveToPosition(115, 81, 1);

                    intake.setServoDown();
                    Thread.sleep(500);


                    intake.intakeOn();
                    Thread.sleep(1000);
                    intake.intakeOff();
                    driveSystem.moveToPosition(120, 84, 1);
                    intake.setServoUp();
                    slide.setTargetLevel(LinearSlide.HeightLevels.Down);
                    Thread.sleep(500);


                    intake.setServoUp();
                    driveSystem.turnTo(190, .5);
                    //Thread.sleep(500);
                    driveSystem.moveToPosition(131, 63, 2);

                    driveSystem.moveToPosition(132, 24, 1);
                    break;

                case NotSeen:
                    telemetry.addData("position", " is Far Left");
                    telemetry.update();
                    driveSystem.moveToPosition(120, 84, 1);

                    slide.setTargetLevel(LinearSlide.HeightLevels.BottomLevel);

                    Thread.sleep(1500);

                    intake.setServoDown();
                    Thread.sleep(500);

                    //this position will vary for different heights on the goal
                    driveSystem.moveToPosition(115, 84, 1);


                    intake.intakeOn();
                    Thread.sleep(1000);
                    intake.intakeOff();
                    driveSystem.moveToPosition(117, 84, 1);
                    intake.setServoUp();
                    slide.setTargetLevel(LinearSlide.HeightLevels.Down);
                    Thread.sleep(500);
                    //following this is unique to carousel and warehouse

                    driveSystem.turnTo(190, .5);
                    //Thread.sleep(500);
                    driveSystem.moveToPosition(131, 63, 2);

                    driveSystem.moveToPosition(132, 24, 1);
                    break;

                case Left:
                    telemetry.addData("position", "is center");
                    telemetry.update();
                    driveSystem.moveToPosition(120, 84, 1);

                    slide.setTargetLevel(LinearSlide.HeightLevels.MiddleLevel);

                    Thread.sleep(1500);

                    intake.setServoDown();
                    Thread.sleep(500);

                    //this position will vary for different heights on the goal
                    driveSystem.moveToPosition(115, 84, 1);


                    intake.intakeOn();
                    Thread.sleep(1000);
                    intake.intakeOff();
                    driveSystem.moveToPosition(117, 84, 1);
                    intake.setServoUp();
                    slide.setTargetLevel(LinearSlide.HeightLevels.Down);
                    Thread.sleep(500);
                    //following this is unique to carousel and warehouse

                    driveSystem.turnTo(190, .5);
                    //Thread.sleep(500);
                    driveSystem.moveToPosition(131, 63, 2);

                    driveSystem.moveToPosition(132, 24, 1);
                    break;


            }
        }
        slide.end();
        odometry.end();

    }
}


