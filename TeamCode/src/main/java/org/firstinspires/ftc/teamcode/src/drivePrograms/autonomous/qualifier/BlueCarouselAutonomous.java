package org.firstinspires.ftc.teamcode.src.drivePrograms.autonomous.qualifier;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.src.robotAttachments.subsystems.LinearSlide;
import org.firstinspires.ftc.teamcode.src.utills.AutoObjDetectionTemplate;
import org.firstinspires.ftc.teamcode.src.utills.MiscUtills;

/**
 * The Autonomous ran on Blue side near spinner for Meet 3
 */
@Autonomous(name = "Blue Carousel Autonomous")
public class BlueCarouselAutonomous extends AutoObjDetectionTemplate {

    @Override
    public void runOpMode() throws InterruptedException {
        try {
            this.initAll();
        } catch (InterruptedException ignored) {
            return;
        } catch (Exception e) {
            telemetry.addData("issue:", MiscUtills.getStackTraceAsString(e));
            telemetry.update();
        }

        leds.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
        MarkerPosition Pos = MarkerPosition.NotSeen;


        odometry.setPosition(133, 101, 270);


        while (!isStarted() && !isStopRequested()) {
            Pos = this.getAverageOfMarker(10, 100);
            telemetry.addData("Position", Pos);
            telemetry.update();
        }


        System.gc();

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
                    intake.setServoClosed();

                    Thread.sleep(2000);

                    //this position will vary for different heights on the goal
                    driveSystem.moveToPosition(115, 81, 1);

                    intake.setServoClosed();
                    Thread.sleep(500);




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

                    intake.setServoClosed();
                    Thread.sleep(500);

                    //this position will vary for different heights on the goal
                    driveSystem.moveToPosition(115, 84, 1);


                    break;
                case Left:
                    telemetry.addData("position", "is left");
                    telemetry.update();
                    driveSystem.moveToPosition(120, 84, 1);


                    slide.setTargetLevel(LinearSlide.HeightLevel.BottomLevel);


                    //TODO: have robot move in closer to load objects on to the hub
                    Thread.sleep(1500);

                    intake.setServoClosed();
                    Thread.sleep(500);

                    //this position will vary for different heights on the goal
                    driveSystem.moveToPosition(115, 84, 1);


                    break;


            }
            //Shared Code

            intake.setIntakeOn();
            Thread.sleep(1000);
            intake.setIntakeOff();
            driveSystem.moveToPosition(117, 84, 1);
            intake.setServoOpen();
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

            boolean warehousePark = false;

            if (warehousePark) {
                //TODO: Make this work
                driveSystem.strafeAtAngle(180, .5);
                Thread.sleep(400);
                odometry.setPosition(144 - 17, 144 - 7, 180);
                driveSystem.moveToPosition(144 - 24, 64, 1, true);
                driveSystem.turnTo(180, 0.5);
                podServos.raise();
                driveSystem.stopAll();
                Thread.sleep(500);
                driveSystem.strafeAtAngle(0, 1);
                Thread.sleep(1750);

            } else {
                driveSystem.moveToPosition(102, 130, 1);
                driveSystem.strafeAtAngle(180, .5);
                Thread.sleep(700);
                driveSystem.stopAll();
            }


        }
        slide.end();
        odometry.end();

    }
}


