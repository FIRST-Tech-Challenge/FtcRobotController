package org.firstinspires.ftc.teamcode.src.drivePrograms.autonomous.qualifier;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.src.robotAttachments.subsystems.LinearSlide;
import org.firstinspires.ftc.teamcode.src.utills.AutoObjDetectionTemplate;
import org.firstinspires.ftc.teamcode.src.utills.MiscUtills;

/**
 * The Autonomous ran on Blue side near warehouse for Meet 3
 */
@Autonomous(name = "Blue Warehouse Autonomous")
public class BlueWarehouseAutonomous extends AutoObjDetectionTemplate {

    @Override
    public void runOpMode() throws InterruptedException {
        try {
            this.initVuforia();
            this.initTfod();
            this.activateTF();
            this.initAll();
        } catch (Exception e) {
            telemetry.addData("issue:", MiscUtills.getStackTraceAsString(e));
            telemetry.update();
        }
        leds.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
        MarkerPosition Pos = MarkerPosition.NotSeen;
        slide.setTargetLevel(LinearSlide.HeightLevel.Down);
        odometry.setPosition(133, 63, 270);

        while (!isStarted()) {
            Pos = this.getAverageOfMarker(10, 100);
            checkStop();

           /* if(Pos != MarkerPosition.NotSeen){
                leds.setPattern(RevBlinkinLedDriver.BlinkinPattern.HOT_PINK);
            }

            */
            telemetry.addData("Position", Pos);
            telemetry.update();
        }

        waitForStart();

        leds.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);

        if (opModeIsActive() && !isStopRequested()) {
            tfod.shutdown();
            vuforia.close();
            System.gc();


            switch (Pos) {
                case Right:
                    telemetry.addData("position", " is far right");
                    telemetry.update();

                    driveSystem.moveToPosition(120, 84, 1);

                    slide.setTargetLevel(LinearSlide.HeightLevel.TopLevel);
                    Thread.sleep(1500);


                    //this position will vary for different heights on the goal
                    driveSystem.moveToPosition(115, 81, 1);

                    intake.setServoClosed();
                    Thread.sleep(500);


                    intake.setIntakeOn();
                    Thread.sleep(1000);
                    intake.setIntakeOff();
                    driveSystem.moveToPosition(120, 84, 1);
                    intake.setServoOpen();
                    slide.setTargetLevel(LinearSlide.HeightLevel.Down);
                    Thread.sleep(500);


                    intake.setServoOpen();
                    driveSystem.turnTo(190, .5);
                    //Thread.sleep(500);
                    driveSystem.moveToPosition(131, 63, 2);

                    driveSystem.moveToPosition(132, 24, 1);
                    break;

                case NotSeen:
                    telemetry.addData("position", " is Far Left");
                    telemetry.update();
                    driveSystem.moveToPosition(120, 84, 1);

                    slide.setTargetLevel(LinearSlide.HeightLevel.BottomLevel);

                    Thread.sleep(1500);

                    intake.setServoClosed();
                    Thread.sleep(500);

                    //this position will vary for different heights on the goal
                    driveSystem.moveToPosition(115, 84, 1);


                    intake.setIntakeOn();
                    Thread.sleep(1000);
                    intake.setIntakeOff();
                    driveSystem.moveToPosition(117, 84, 1);
                    intake.setServoOpen();
                    slide.setTargetLevel(LinearSlide.HeightLevel.Down);
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

                    slide.setTargetLevel(LinearSlide.HeightLevel.MiddleLevel);

                    Thread.sleep(1500);

                    intake.setServoClosed();
                    Thread.sleep(500);

                    //this position will vary for different heights on the goal
                    driveSystem.moveToPosition(115, 84, 1);


                    intake.setIntakeOn();
                    Thread.sleep(1000);
                    intake.setIntakeOff();
                    driveSystem.moveToPosition(117, 84, 1);
                    intake.setServoOpen();
                    slide.setTargetLevel(LinearSlide.HeightLevel.Down);
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


