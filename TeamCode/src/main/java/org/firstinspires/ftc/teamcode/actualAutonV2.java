package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(group="chrisBot", name="actualAutonV2")

public class actualAutonV2 extends LinearOpMode {

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

        // Drive to first foam tile

        double inches = -36;

        telemetry.addLine("Driving " + inches + " inches...");
        telemetry.update();

        robot.encoderDrive(inches);

        // leave goal

        pause(1000);

        telemetry.addLine("Moving left");
        telemetry.update();
        robot.wheelMecanumDrive(robot.calculateInches(-16,0), 0.7);

        // Line up with first target

        telemetry.addLine("Lining up with first Powershot target...");
        telemetry.update();

        robot.encoderDrive(-12);

        for(int i = 0; i < 3; i++) {
            telemetry.addLine("Shoot!");
            telemetry.update();

            robot.liftUp();
            robot.autonShoot();
            robot.liftDown();

            telemetry.addLine("Moving over...");
            telemetry.update();

            robot.wheelMecanumDrive(robot.calculateInches(-7.5,0),0.7);

            pause(1000);
        }

        // Drive backward to go over white line

        telemetry.addLine("Moving over white line...");
        telemetry.update();

        robot.encoderDrive(0.5,-5);

        pause(1000);


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
