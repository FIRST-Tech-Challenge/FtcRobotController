package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(group="chrisBot", name="actualAutonRingsOnly")

public class actualAutonRingsOnly extends LinearOpMode {

    chrisBot robot = new chrisBot();

    boolean debugWait = true;

    private ElapsedTime time = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap, telemetry);

        waitForStart();

        telemetry.clear();
        robot.breakTelemetry();
        telemetry.addLine("Started match");
        telemetry.update();
        time.reset();

        // Drive forward appropriate amount to be able to clearly see the rings

        telemetry.addLine("Driving forward...");
        telemetry.update();

        robot.encoderDrive(-24);

        pause(1000);

        // Sense the number of rings using TensorFlow (demo code)
        // Activate TensorFlow Object Detection before we wait for the start command.
        // Do it here so that the Camera Stream window will have the TensorFlow annotations visible.

        if (robot.tfod != null) { robot.tfod.activate(); }
        boolean[] ringPositions;
        ringPositions = robot.detectRings();


        // Create boolean variables to store the amount of rings
        boolean oneRing = ringPositions[0];
        boolean fourRings = ringPositions[1];
        boolean noRings = !oneRing && !fourRings;

        telemetry.addData("One ring", oneRing);
        telemetry.addData("Four ring", fourRings);
        telemetry.update();

        pause(1000);

        // Drive to the correct foam tile foam tile

        double inches = -48;

        if (oneRing) { inches += -24; }
        else if (fourRings) { inches += -48; }

        telemetry.addData("Inches to drive",inches);
        telemetry.update();

        pause(2000);

        telemetry.addLine("Driving the above number of inches...");
        telemetry.update();

        robot.encoderDrive(inches);

        // leave goal

        pause(1000);

        telemetry.addLine("Moving back");
        telemetry.update();

        inches = 0;
        if (oneRing) { inches += 24; }
        else if (fourRings) { inches += 48; }

        robot.encoderDrive(inches);

        telemetry.addData("Auton done, time elapsed (ms)",time.milliseconds());
        telemetry.update();

        // End auton

        robot.tfod.shutdown();

        sleep(60000);

    }

    private void pause(long ms) {
        if (debugWait) {
            sleep(ms);
        }
    }
}
