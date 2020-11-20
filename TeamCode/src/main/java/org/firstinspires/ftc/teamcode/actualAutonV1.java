// See Auton_plan.md for more info

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(group="Zippo", name="actualAutonV1")

public class actualAutonV1 extends LinearOpMode {

    chrisBot robot = new chrisBot();

    public void say(String s) {
        telemetry.addLine(s);
        telemetry.update();
    }
    public int[] range(int start, int end) {
        if (end - start <= 0) {
            return new int[]{};
        }
        else {
            int[] arr = new int[end - start];
            for(int i = start; i < end; i++) {
                arr[i-start] = i;
            }
            return arr;
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {

        // Start with wobble goal in hand (left) and 3 rings

        // Initialize robot
        robot.init(hardwareMap);
        robot.initVuforia();
        robot.initTfod();
        robot.setAllPower(0);

        say("Please wait for encoders to reset");

        // Wait for the game to start (driver presses PLAY)

        waitForStart();

        // Drive forward appropriate amount to be able to clearly see the rings

        robot.encoderDrive(24);

        // Sense the number of rings using TensorFlow (demo code)
        // Activate TensorFlow Object Detection before we wait for the start command.
        // Do it here so that the Camera Stream window will have the TensorFlow annotations visible.

        if (robot.tfod != null) { robot.tfod.activate(); }

        // Create boolean variables to store the amount of rings

        boolean[] ringPositions = robot.detectRings();
        boolean oneRing = ringPositions[0];
        boolean fourRings = ringPositions[1];
        boolean noRings = !oneRing && !fourRings;

        sleep(1000);

        // Drive to first foam tile
        // (conditional) 1 ring: drive 24 inches forward to second foam tile
        // (conditional) 4 rings: drive 48 inches forward to third foam tile
        // Calculate the number of inches to drive forward in one step

        double inches = 36;

        if (oneRing) { inches += 24; }
        else if (fourRings) { inches += 48; }

        robot.encoderDrive(inches);

        // (conditional) 1 ring: strafe right 18 inches
        // (else) drop goal

        if (oneRing) { robot.wheelMecanumDrive(robot.calculateInches(24,0)); }

        robot.dropGoal(); /* DUMMY METHOD */

        // Unify positions between cases (programming trick)

        if(fourRings) { robot.encoderDrive(-24); }
        else if(noRings) { robot.encoderDrive(24); }

        if(!oneRing) { robot.wheelMecanumDrive(robot.calculateInches(24,0)); }

        // Line up with first target

        robot.wheelMecanumDrive(robot.calculateInches(18,-40));

        // Shoot first target

        robot.shoot();

        // Repeat with other two targets

        for(int i : range(0,2)) {
            robot.wheelMecanumDrive(robot.calculateInches(24,0));
            robot.shoot();
        }

        // Drive forward to go over white line

        robot.wheelMecanumDrive(robot.calculateInches(0,16));

        // End auton

    }
}
