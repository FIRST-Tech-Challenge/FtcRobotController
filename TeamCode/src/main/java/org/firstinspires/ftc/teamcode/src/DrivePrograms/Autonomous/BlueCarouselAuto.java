package org.firstinspires.ftc.teamcode.src.DrivePrograms.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.src.Utills.AutoObjDetectionTemplate;
import org.firstinspires.ftc.teamcode.src.robotAttachments.Sensors.RobotVoltageSensor;
import org.firstinspires.ftc.teamcode.src.robotAttachments.Subsystems.LinearSlide;


@Autonomous(name = "BlueCarouselAuto")
public class BlueCarouselAuto extends AutoObjDetectionTemplate {
    LinearSlide slide;

    @Override
    public void runOpMode() throws InterruptedException {
        this.initAll();
        MarkerPosition Pos = MarkerPosition.NotSeen;
        RobotVoltageSensor s = new RobotVoltageSensor(hardwareMap);
        slide = new LinearSlide(hardwareMap, "slide_motor", s, this::opModeIsActive, this::isStopRequested);
        Thread t = new Thread(slide);
        slide.setTargetLevel(LinearSlide.HeightLevels.Down);
        this.initVuforia();
        this.initTfod();
        this.activate();

        while (!isStarted()) {
            Pos = this.getAverageOfMarker(10, 100);
            telemetry.addData("Position", Pos);
            telemetry.update();
        }


        odometry.setPosition(133, 101, 270);

        waitForStart();
        tfod.shutdown();
        vuforia.close();
        System.gc();
        t.start();


        switch (Pos) {
            case NotSeen:
                telemetry.addData("position", " is far right");
                telemetry.update();

                driveSystem.moveToPosition(120, 84, 1);


                slide.setTargetLevel(LinearSlide.HeightLevels.TopLevel);
                intake.setServoDown();

                Thread.sleep(500);
                //TODO: have robot move in closer to load objects on to the hub
                //driveSystem.moveToPosition(111, 84, 1);
                Thread.sleep(500);
                intake.intakeOn();
                Thread.sleep(1000);
                intake.intakeOff();
                driveSystem.moveToPosition(120, 84, 1);
                intake.setServoUp();

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


                slide.setTargetLevel(LinearSlide.HeightLevels.MiddleLevel);
                intake.setServoDown();

                Thread.sleep(500);
                //TODO: have robot move in closer to load objects on to the hub
                //driveSystem.moveToPosition(111, 84, 1);
                Thread.sleep(500);
                intake.intakeOn();
                Thread.sleep(1000);
                intake.intakeOff();
                driveSystem.moveToPosition(120, 84, 1);
                intake.setServoUp();

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


                slide.setTargetLevel(LinearSlide.HeightLevels.BottomLevel);
                intake.setServoDown();

                Thread.sleep(500);
                //TODO: have robot move in closer to load objects on to the hub
                //driveSystem.moveToPosition(111, 84, 1);
                Thread.sleep(500);
                intake.intakeOn();
                Thread.sleep(1000);
                intake.intakeOff();
                driveSystem.moveToPosition(120, 84, 1);
                intake.setServoUp();

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
        telemetry.update();
        while (opModeIsActive() && !isStopRequested()) ;

    }
}


