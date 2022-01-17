package org.firstinspires.ftc.teamcode.src.drivePrograms.autonomous.qualifier;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver.BlinkinPattern;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.src.robotAttachments.subsystems.LinearSlide;
import org.firstinspires.ftc.teamcode.src.utills.AutoObjDetectionTemplate;
import org.firstinspires.ftc.teamcode.src.utills.enums.BarcodePositions;

/**
 * The Autonomous ran on Blue side near warehouse for Qualifier
 */
@Autonomous(name = "Blue Warehouse Autonomous")
public class BlueWarehouseAutonomous extends AutoObjDetectionTemplate {
    static final BlinkinPattern def = BlinkinPattern.BLUE;
    static final double[] initialPos = {133, 63, 0};

    @Override
    public void opModeMain() throws InterruptedException {
        this.initAll();
        leds.setPattern(def);
        odometry.setPosition(initialPos[0], initialPos[1], initialPos[2]);

        telemetry.addData("GC", "Started");
        telemetry.update();
        System.gc();
        telemetry.addData("GC", "Finished");
        telemetry.update();

        BarcodePositions Pos;
        do {
            Pos = this.getAverageOfMarker(10, 100);
            telemetry.addData("Position", Pos);
            telemetry.update();
        } while (!isStarted() && !isStopRequested());

        waitForStart();

        if (opModeIsActive() && !isStopRequested()) {
            tfod.shutdown();
            vuforia.close();

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


