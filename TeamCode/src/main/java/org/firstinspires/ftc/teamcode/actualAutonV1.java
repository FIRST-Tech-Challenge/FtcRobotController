package org.firstinspires.ftc.teamcode;

// See Auton_plan.md for more info

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

@Autonomous(group="Zippo", name="actualAutonV1")

public class actualAutonV1 extends LinearOpMode {

    // Debug values
    private static final boolean debugMode = false;
    private static boolean debugWait = true, debugRings = false;
    private static boolean[] ringValues = {false, false};

    private ElapsedTime time = new ElapsedTime();

    chrisBot robot = new chrisBot();

    WebcamName webcam;

    @Override
    public void runOpMode() {

        // Start with wobble goal in hand (left) and 3 rings

        // Initialize robot
        robot.init(hardwareMap, telemetry, true, true);

        // Wait for the game to start (driver presses PLAY)

        waitForStart();

        robot.breakTelemetry();

        // Debug settings
        if (debugMode) {
            Telemetry.Item t = telemetry.addData("Wait for debug purposes",debugWait);
            telemetry.update();
            while(!gamepad1.a && opModeIsActive()) {
                if(gamepad1.dpad_left || gamepad1.dpad_right) {
                    debugWait = !debugWait;
                }
                t.setValue(debugWait);
                telemetry.update();
                sleep(500);
            }
            telemetry.clear();
            Telemetry.Item q = telemetry.addData("Debug ring values",debugRings);
            telemetry.update();
            while(!gamepad1.a && opModeIsActive()) {
                if(gamepad1.dpad_left || gamepad1.dpad_right) {
                    debugWait = !debugRings;
                }
                telemetry.update();
                q.setValue(debugRings);
                sleep(500);
            }
            telemetry.clear();
            if (debugRings) {
                Telemetry.Item r = telemetry.addData("Set debug value for rings",ringValues);
                telemetry.update();
                while(!gamepad1.a && opModeIsActive()) {
                    if(gamepad1.dpad_left) {
                        if(ringValues[0] == ringValues[1]) {
                            ringValues = new boolean[]{false, true};
                        }
                        else if(ringValues[1]) {
                            ringValues = new boolean[]{true, false};
                        }
                        else {
                            ringValues = new boolean[]{false, false};
                        }
                    }
                    if(gamepad1.dpad_right) {
                        if(ringValues[0] == ringValues[1]) {
                            ringValues = new boolean[]{true, false};
                        }
                        else if(ringValues[0]) {
                            ringValues = new boolean[]{false, true};
                        }
                        else {
                            ringValues = new boolean[]{false, false};
                        }
                    }
                    r.setValue(ringValues);
                    telemetry.update();
                    sleep(500);
                }
            }
        }

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

        if (debugRings) {
            ringPositions = ringValues;
        } else {
            ringPositions = robot.detectRings();
        }

        // Create boolean variables to store the amount of rings
        boolean oneRing = ringPositions[0];
        boolean fourRings = ringPositions[1];
        boolean noRings = !oneRing && !fourRings;

        pause(1000);

        // Drive to first foam tile
        // (conditional) 1 ring: drive 24 inches forward to second foam tile
        // (conditional) 4 rings: drive 48 inches forward to third foam tile
        // Calculate the number of inches to drive forward in one step

        double inches = -48;

        if (oneRing) { inches += -24; }
        else if (fourRings) { inches += -48; }

        telemetry.addData("Inches to drive",inches);
        telemetry.update();

        pause(2000);

        telemetry.addLine("Driving the above number of inches...");
        telemetry.update();

        robot.encoderDrive(inches);

        // (conditional) 1 ring: strafe right 18 inches
        // (else) drop goal

        pause(1000);

        if (oneRing) {
            telemetry.addLine("Moving into \"one-ring\" square...");
            telemetry.update();
            robot.wheelMecanumDrive(robot.calculateInches(-24,0), 0.7);
        }

        robot.dropGoal(); /* DUMMY METHOD */

        pause(1000);

        // Unify positions between cases (programming trick)

        telemetry.addLine("Driving to unify positions on the field...");
        telemetry.update();

        if(fourRings) { robot.encoderDrive(24); }
        else if(noRings) { robot.encoderDrive(-24); }

        if(!oneRing) {
            robot.wheelMecanumDrive(robot.calculateInches(-24,0), 0.7);
            telemetry.addLine("Moved over");
            telemetry.update();
        }

        pause(1000);

        // Line up with first target

        telemetry.addLine("Lining up with first Powershot target...");
        telemetry.update();

        robot.wheelMecanumDrive(robot.calculateInches(-18,52),0.7);

        // Shoot first target

        telemetry.addLine("Shoot!");
        telemetry.update();

        robot.intakeOn();
        robot.shootOn();

        pause(1000);

        // Repeat with other two targets


        for(int i : chrisBot.range(0,2)) {
            telemetry.addLine("Moving over...");
            telemetry.update();
            robot.wheelMecanumDrive(robot.calculateInches(-7.5,0),0.7);

            pause(1000);
        }

        robot.intakeOff();
        robot.shootOff();

        // Drive forward to go over white line

        telemetry.addLine("Moving over white line...");
        telemetry.update();

        robot.wheelMecanumDrive(robot.calculateInches(0,-16));

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
