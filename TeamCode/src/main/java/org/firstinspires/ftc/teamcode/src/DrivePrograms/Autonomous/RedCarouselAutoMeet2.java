package org.firstinspires.ftc.teamcode.src.DrivePrograms.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.src.Utills.AutoObjDetectionTemplate;
import org.firstinspires.ftc.teamcode.src.robotAttachments.Sensors.RobotVoltageSensor;
import org.firstinspires.ftc.teamcode.src.robotAttachments.Subsystems.LinearSlide;

@Autonomous(name = "RedCarouselMeet2")
public class RedCarouselAutoMeet2 extends AutoObjDetectionTemplate {
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
        odometry.setPosition(7, 101, 90);

        waitForStart();
        tfod.shutdown();
        vuforia.close();
        System.gc();
        t.start();


        switch (Pos) {
            case NotSeen:
                telemetry.addData("position", " is far left");
                telemetry.update();
                slide.setTargetLevel(LinearSlide.HeightLevels.TopLevel);
                driveSystem.moveToPosition(20, 84, 1);

                //TODO: create movements to raise linearslide, drop bucket, move forward and unload objects into the goal
                Thread.sleep(500);
                slide.setTargetLevel(LinearSlide.HeightLevels.Down);
                driveSystem.moveToPosition(20, 130, 1);

                //strafe at angle into the wall and then the carousel
                driveSystem.strafeAtAngle(270, .5);
                Thread.sleep(500);
                driveSystem.strafeAtAngle(180, .5);
                Thread.sleep(700);
                driveSystem.stopAll();

                //spin off the duck
                spinner.spinOffRedDuck();

                // park
                driveSystem.moveToPosition(33, 130, 1);
                driveSystem.strafeAtAngle(270, .5);
                Thread.sleep(300);
                driveSystem.stopAll();

                break;
            case Right:
                telemetry.addData("position", " is right");
                telemetry.update();
                driveSystem.moveToPosition(20, 85, 1);

                slide.setTargetLevel(LinearSlide.HeightLevels.MiddleLevel);
                //TODO: create movements to raise linearslide, drop bucket, move forward and unload objects into the goal


                break;
            case Left:
                telemetry.addData("position", "is center");
                telemetry.update();
                driveSystem.moveToPosition(20, 85, 1);

                slide.setTargetLevel(LinearSlide.HeightLevels.BottomLevel);
                //TODO: create movements to raise linearslide, drop bucket, move forward and unload objects into the goal


                break;


        }
        telemetry.update();
        while (opModeIsActive() && !isStopRequested()) ;

    }
}
