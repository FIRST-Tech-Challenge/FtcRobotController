package org.firstinspires.ftc.teamcode.src.DrivePrograms.Autonomous;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.src.Utills.AutoObjDetectionTemplate;
import org.firstinspires.ftc.teamcode.src.robotAttachments.Subsystems.LinearSlide;


@Autonomous(name = "Blue Carousel Autonomous")
public class BlueCarouselAutonomous extends AutoObjDetectionTemplate {

    @Override
    public void runOpMode() throws InterruptedException {
        this.initAll();
        leds.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
        MarkerPosition Pos = MarkerPosition.NotSeen;
        this.initVuforia();
        this.initTfod();
        this.activate();

        odometry.setPosition(133, 101, 270);

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
                case NotSeen:
                    telemetry.addData("position", " is far right");
                    telemetry.update();

                    driveSystem.moveToPosition(120, 84, 1);


                    slide.setTargetLevel(LinearSlide.HeightLevel.TopLevel);
                    intake.setServoDown();

                    Thread.sleep(2000);

                    //this position will vary for different heights on the goal
                    driveSystem.moveToPosition(115, 81, 1);

                    intake.setServoDown();
                    Thread.sleep(500);


                    intake.intakeOn();
                    Thread.sleep(1000);
                    intake.intakeOff();
                    driveSystem.moveToPosition(120, 84, 1);
                    intake.setServoUp();
                    slide.setTargetLevel(LinearSlide.HeightLevel.Down);
                    Thread.sleep(500);


                    driveSystem.moveToPosition(126, 112, 1);
                    driveSystem.turnTo(185, .5);
                    driveSystem.moveToPosition(116, 130, 1);
                    driveSystem.strafeAtAngle(180, .5);
                    Thread.sleep(400);
                    driveSystem.stopAll();
                    driveSystem.strafeAtAngle(270, .55);
                    Thread.sleep(700);
                    driveSystem.stopAll();
                    spinner.spinOffBlueDuck();
                    driveSystem.moveToPosition(102, 130, 1);
                    driveSystem.strafeAtAngle(180, .5);
                    Thread.sleep(700);
                    driveSystem.stopAll();

                /*driveSystem.moveToPosition(126, 1, 1.5, true);
                driveSystem.strafeAtAngle(180 + 35, .5);
                Thread.sleep(500);
                spinner.spinOffBlueDuck();
                driveSystem.moveToPosition(105, 132, 1);
                driveSystem.strafeAtAngle(180, 0.5);
                Thread.sleep(1000);

                 */


                    break;
                case Right:
                    telemetry.addData("position", " is center");
                    telemetry.update();
                    driveSystem.moveToPosition(120, 84, 1);


                    slide.setTargetLevel(LinearSlide.HeightLevel.MiddleLevel);

                    //TODO: have robot move in closer to load objects on to the hub


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
                    slide.setTargetLevel(LinearSlide.HeightLevel.Down);
                    Thread.sleep(500);
                    //following this is unique to carousel and warehouse


                    driveSystem.moveToPosition(126, 112, 1);
                    driveSystem.turnTo(185, .5);
                    driveSystem.moveToPosition(116, 130, 1);
                    driveSystem.strafeAtAngle(180, .5);
                    Thread.sleep(400);
                    driveSystem.stopAll();
                    driveSystem.strafeAtAngle(270, .55);
                    Thread.sleep(700);
                    driveSystem.stopAll();
                    spinner.spinOffBlueDuck();
                    driveSystem.moveToPosition(102, 130, 1);
                    driveSystem.strafeAtAngle(180, .5);
                    Thread.sleep(700);
                    driveSystem.stopAll();


                    break;
                case Left:
                    telemetry.addData("position", "is left");
                    telemetry.update();
                    driveSystem.moveToPosition(120, 84, 1);


                    slide.setTargetLevel(LinearSlide.HeightLevel.BottomLevel);


                    //TODO: have robot move in closer to load objects on to the hub
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
                    slide.setTargetLevel(LinearSlide.HeightLevel.Down);
                    Thread.sleep(500);
                    //following this is unique to carousel and warehouse


                    driveSystem.moveToPosition(126, 112, 1);
                    driveSystem.turnTo(185, .5);
                    driveSystem.moveToPosition(116, 130, 1);
                    driveSystem.strafeAtAngle(180, .5);
                    Thread.sleep(400);
                    driveSystem.stopAll();
                    driveSystem.strafeAtAngle(270, .55);
                    Thread.sleep(700);
                    driveSystem.stopAll();
                    spinner.spinOffBlueDuck();
                    driveSystem.moveToPosition(102, 130, 1);
                    driveSystem.strafeAtAngle(180, .5);
                    Thread.sleep(700);
                    driveSystem.stopAll();


                    break;


            }
        }
        slide.end();
        odometry.end();

    }
}


