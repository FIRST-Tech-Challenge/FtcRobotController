package org.firstinspires.ftc.teamcode;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Scheduler;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "Auto", group = "Autonomous")
public class Auto extends LinearOpMode {
    boolean red = false;
    boolean blue = false;
    boolean close = false;
    boolean far = false;
    boolean colorSelected = false;
    boolean locationSelected = false;
    boolean confirmed = false;
    boolean telemetryFix = false;


    @Override
    public void runOpMode() throws InterruptedException {
        // declare subassembly classes
        // subassembly class

        // name subassembly classes
        // name

        autoBlueFar ABFar = new autoBlueFar();

        telemetry.addLine("press X for blue, press B for red 'w'");
        telemetry.update();

        while (!colorSelected) {
            if (gamepad1.bWasReleased()) {
                red = true;
                colorSelected = true;
            } else {
                if (gamepad1.xWasReleased()) {
                    blue = true;
                    colorSelected = true;
                }
            }
        }
        telemetry.addLine("press Up for Far, press Down for Close");
        telemetry.update();

        while (!locationSelected) {
            if (gamepad1.dpadDownWasReleased()) {
                close = true;
                locationSelected = true;
            } else {
                if (gamepad1.dpadUpWasReleased()) {
                    far = true;
                    locationSelected = true;
                }
            }


            waitForStart();

            while (opModeIsActive()) {
                while (!confirmed) {
                    if (blue && far) {
                        telemetry.addLine("blue left selected :]");
                        confirmed = true;
                    } else if (blue && close) {
                        telemetry.addLine("blue right selected :]");
                        confirmed = true;
                    } else if (red && far) {
                        telemetry.addLine("red left without saying goodbye :(");
                        confirmed = true;
                    } else {
                        telemetry.addLine("red isnt right because blue is a better color but ok i guess :/");
                        confirmed = true;
                    }
                }
                if (!telemetryFix) {
                    telemetry.addLine("auto started wow");
                    telemetry.update();
                    telemetryFix = true;
                }
            }
            if (blue && far) {
//                ABFar.autoRoutine();
                Follower follower = ABFar.follower;


                Scheduler.reset();
                follower = Constants.create(hardwareMap);
                follower.setPose(ABFar.startBlueFar);
                schedule(follow(follower, ABFar.launchToFlowerABFar()));
                schedule(follow(follower, ABFar.flowerToLaunch()));
                schedule(follow(follower, ABFar.launchToFlower2()));
                schedule(follow(follower, ABFar.flowerToLaunch2()));
                schedule(follow(follower, ABFar.launchToPark()));

                schedule(ABFar.autoBlueFar());

                while (opModeIsActive()) {
                    follower.update();
                    Scheduler.execute();
                }
            }
        }
    }
}
