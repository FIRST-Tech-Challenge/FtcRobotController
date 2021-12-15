package org.firstinspires.ftc.teamcode.src.DrivePrograms.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.src.Utills.AutoObjDetectionTemplate;
import org.firstinspires.ftc.teamcode.src.robotAttachments.Subsystems.LinearSlide;

@Autonomous(name = "RedCarouselMeet2")
public class RedCarouselAutoMeet2 extends AutoObjDetectionTemplate {

    @Override
    public void runOpMode() throws InterruptedException {
        this.initAll();
        MarkerPosition Pos = MarkerPosition.NotSeen;
        slide.setTargetLevel(LinearSlide.HeightLevels.Down);
        this.initVuforia();
        this.initTfod();
        this.activate();

        odometry.setPosition(7, 101, 90);

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
                    telemetry.addData("position", " is right");
                    telemetry.update();
                    driveSystem.moveToPosition(20, 84, 1);
                    slide.setTargetLevel(LinearSlide.HeightLevels.TopLevel);

                    //TODO: create movements to raise linearslide, drop bucket, move forward and unload objects into the goal
                    Thread.sleep(1500);


                    //this position will vary for different heights on the goal
                    driveSystem.moveToPosition(25, 81, 1);

                    intake.setServoDown();
                    Thread.sleep(500);

                    intake.intakeOn();
                    Thread.sleep(1000);
                    intake.intakeOff();
                    driveSystem.moveToPosition(23, 84, 1);
                    intake.setServoUp();
                    Thread.sleep(500);
                    slide.setTargetLevel(LinearSlide.HeightLevels.Down);
                    Thread.sleep(500);
                    //following this is unique to carousel and warehouse

                    driveSystem.moveToPosition(20, 130, 1);

                    //strafe at angle into the wall and then the carousel
                    driveSystem.strafeAtAngle(270, .5);
                    Thread.sleep(500);
                    driveSystem.strafeAtAngle(180, .5);
                    Thread.sleep(650);
                    driveSystem.stopAll();

                    //spin off the duck
                    spinner.spinOffRedDuck();

                    // park
                    driveSystem.moveToPosition(31.5, 130, 1);
                    driveSystem.strafeAtAngle(270, .5);
                    Thread.sleep(300);
                    driveSystem.stopAll();

                    break;
                case NotSeen:
                    telemetry.addData("position", " far left");
                    telemetry.update();
                    driveSystem.moveToPosition(20, 85, 1);

                    slide.setTargetLevel(LinearSlide.HeightLevels.BottomLevel);

                    //TODO: create movements to raise linearslide, drop bucket, move forward and unload objects into the goal

                    Thread.sleep(1500);

                    intake.setServoDown();
                    Thread.sleep(500);

                    //this position will vary for different heights on the goal
                    driveSystem.moveToPosition(25, 84, 1);


                    intake.intakeOn();
                    Thread.sleep(1000);
                    intake.intakeOff();
                    driveSystem.moveToPosition(23, 84, 1);
                    intake.setServoUp();
                    slide.setTargetLevel(LinearSlide.HeightLevels.Down);
                    Thread.sleep(500);
                    //following this is unique to carousel and warehouse
                    driveSystem.moveToPosition(20, 130, 1);

                    //strafe at angle into the wall and then the carousel
                    driveSystem.strafeAtAngle(270, .5);
                    Thread.sleep(500);
                    driveSystem.strafeAtAngle(180, .5);
                    Thread.sleep(650);
                    driveSystem.stopAll();

                    //spin off the duck
                    spinner.spinOffRedDuck();

                    // park
                    driveSystem.moveToPosition(31.5, 130, 1);
                    driveSystem.strafeAtAngle(270, .5);
                    Thread.sleep(300);
                    driveSystem.stopAll();


                    break;
                case Left:
                    telemetry.addData("position", "is center");
                    telemetry.update();
                    driveSystem.moveToPosition(20, 85, 1);

                    slide.setTargetLevel(LinearSlide.HeightLevels.MiddleLevel);
                    //TODO: create movements to raise linearslide, drop bucket, move forward and unload objects into the goal

                    Thread.sleep(1500);

                    intake.setServoDown();
                    Thread.sleep(500);

                    //this position will vary for different heights on the goal
                    driveSystem.moveToPosition(25, 84, 1);


                    intake.intakeOn();
                    Thread.sleep(1000);
                    intake.intakeOff();
                    driveSystem.moveToPosition(23, 84, 1);
                    intake.setServoUp();
                    slide.setTargetLevel(LinearSlide.HeightLevels.Down);
                    Thread.sleep(500);
                    //following this is unique to carousel and warehouse

                    driveSystem.moveToPosition(20, 130, 1);

                    //strafe at angle into the wall and then the carousel
                    driveSystem.strafeAtAngle(270, .5);
                    Thread.sleep(500);
                    driveSystem.strafeAtAngle(180, .5);
                    Thread.sleep(650);
                    driveSystem.stopAll();

                    //spin off the duck
                    spinner.spinOffRedDuck();

                    // park
                    driveSystem.moveToPosition(31.2, 130, 1);
                    driveSystem.strafeAtAngle(270, .5);
                    Thread.sleep(300);
                    driveSystem.stopAll();


                    break;


            }
        }
        slide.end();
        odometry.end();
    }
}
