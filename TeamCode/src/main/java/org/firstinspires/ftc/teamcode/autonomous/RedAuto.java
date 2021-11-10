/* Copyright (c) 2019 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.drive.MecanumDrive6340;
import org.firstinspires.ftc.teamcode.drive.advanced.PoseStorage;

import java.util.List;

/**
 * This 2020-2021 OpMode illustrates the basics of using the TensorFlow Object Detection API to
 * determine the position of the Ultimate Goal game elements.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 *
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained below.
 */
@Autonomous(name = "Red Auto", group = "Autonomous")
//@Disabled

public class RedAuto extends LinearOpMode {

    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";
    private String targetZone = "A";

    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
    private static final String VUFORIA_KEY =
            "AZWzerv/////AAABmZeKo4MkD08MoSz5oHB/JU6N1BsUWpfHgQeAeVZemAypSUGVQhvAHo6+v7kJ3MITd8530MhwxRx7GjRtdCs1qjPmdKiJK66dv0yN4Zh4NvKBfP5p4TJjM+G0GoMVgVK0pItm2U56/SVqQH2AYtczQ+giw6zBe4eNhHPJCMY5C2t5Cs6IxxjZlMkRF85l8YAUlKGLipnoZ1T/mX8DNuThQA57qsIB2EN6pGWe8GI64hcPItQ0j7Oyjp82lEN13rYQYsS3Ur4a6//D6yhwa0rogXAysG68G+VgC1mNlj1CjX60qDI84ZN0b/A081xXqjeyFqZK8A/jO8y7BGz9ZuuZNxxXIon6xRNeKYudpfTD23+5";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;

    @Override
    public void runOpMode() {

/*
TRAJECTORIES
 */
        MecanumDrive6340 drive = new MecanumDrive6340(hardwareMap);

        Pose2d startPose = new Pose2d(-62,-55, Math.toRadians(0));

        drive.setPoseEstimate(startPose);

        Trajectory shooterLine = drive.trajectoryBuilder(startPose)
                .splineTo(new Vector2d( -17,-55), Math.toRadians(0))
                .build();



//A STUFF
        Trajectory lineToA = drive.trajectoryBuilder(shooterLine.end())
                .splineTo(new Vector2d( 8,-50), Math.toRadians(0))
                .build();
        Trajectory aToGoal = drive.trajectoryBuilder(lineToA.end(), true )
                .splineTo(new Vector2d(-33, -24), Math.toRadians(180))
                .build();
        Trajectory goalToA = drive.trajectoryBuilder(aToGoal.end(),true)
                .splineTo(new Vector2d(3, -45), Math.toRadians(270))
                .build();
        Trajectory aLine = drive.trajectoryBuilder(goalToA.end())
                .splineTo(new Vector2d(8, -6), Math.toRadians(90))
                .build();



        //B STUFFF
        Trajectory startToB = drive.trajectoryBuilder(shooterLine.end())
                .forward(48)
                .build();
        Trajectory left = drive.trajectoryBuilder(startToB.end())
                .strafeLeft(33)
                .build();
        Trajectory bToGoal = drive.trajectoryBuilder(left.end(), true)
                .splineTo(new Vector2d(-33, -24),Math.toRadians(180))
                .build();

        Trajectory goalToB = drive.trajectoryBuilder(bToGoal.end(),true)
                .splineTo(new Vector2d(25 , -17), Math.toRadians(270))
                .build();

        Trajectory bLine2 = drive.trajectoryBuilder(goalToB.end())
                .splineTo(new Vector2d( 5, 5 ), Math.toRadians(180))
                .build();

//C STUFF
        Trajectory targetZoneC = drive.trajectoryBuilder(shooterLine.end())
                .splineTo(new Vector2d(55, -51), Math.toRadians(0))
                .build();

        Trajectory cToGoal = drive.trajectoryBuilder(targetZoneC.end(),true)
                .splineTo(new Vector2d(-33,-22), Math.toRadians(180))
                .build();

        Trajectory goalToC = drive.trajectoryBuilder(cToGoal.end(), true)
                .splineTo(new Vector2d(48,-45), Math.toRadians(270))
                .build();
        Trajectory cToLine = drive.trajectoryBuilder(goalToC.end())
                .splineTo(new Vector2d(8,12), Math.toRadians(180))
                .addTemporalMarker(0.5,()-> {

                })
                .build();






        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia();
        initTfod();

        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        if (tfod != null) {
            tfod.activate();

            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can adjust the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 16/9).
            tfod.setZoom(2.5, 16.0/9.0);
        }
drive.shooterServo.setPosition(0); //Hold

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {
            ElapsedTime timeout = new ElapsedTime();
            while (timeout.time()<2.0) {
                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                      telemetry.addData("# Object Detected", updatedRecognitions.size());


                      // step through the list of recognitions and display boundary info.
                      int i = 0;
                      for (Recognition recognition : updatedRecognitions) {
                        telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                        telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                recognition.getLeft(), recognition.getTop());
                        telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                recognition.getRight(), recognition.getBottom());

                        // check label to see which target zone to go after
                        if (recognition.getLabel().equals("Single")) {
                            telemetry.addData("Target Zone", "B");
                            targetZone = "B";
                        } else if (recognition.getLabel().equals ("Quad")) {
                            telemetry.addData("Target Zone", "C");
                            targetZone = "C";
                        }


                      }
                    }
                }
                telemetry.update();

            }
        }

        if (tfod != null) {
            tfod.shutdown();
        }

        //AUTONOMOUS STUFF RIGHT HERE

        if (targetZone.equals("C")) {

            drive.grabGoal();//grab goal,
            sleep(500);
            drive.shooter.setVelocity(1550);
            drive.followTrajectory(shooterLine);
            telemetry.update();

            drive.shooterServo.setPosition(1);
            sleep(1000);
            drive.shooterServo.setPosition(0);
            sleep(1000);
            drive.shooterServo.setPosition(1);
            sleep(1000);
            drive.shooterServo.setPosition(0);
            sleep(1000);
            drive.shooterServo.setPosition(1);
            sleep(1000);
            drive.shooterServo.setPosition(0);
            sleep(1000);
            drive.followTrajectory(targetZoneC); //Drive to target zone C
            drive.releaseGoal();// release goal,
            sleep(1000);// wait 1 sec,
            drive.deployArm();// deploy arm,
            drive.shooter.setPower(0);
            drive.armServo.setPosition(.7);
            drive.followTrajectory(cToGoal);// drive to second goal,
            drive.grabGoal();// grab goal,
            sleep(1000);
            drive.followTrajectory(goalToC);// Drive back to C
            drive.releaseGoal();// release goal
            sleep(100);
            drive.followTrajectory(cToLine);// drive to line + pick up arm
            drive.retractArm();
            sleep(1000);

             } else if (targetZone.equals("B")) {

            drive.grabGoal();
            sleep(500);
            drive.shooter.setVelocity(1600);
            drive.followTrajectory(shooterLine);
            telemetry.update();
            drive.shooterServo.setPosition(1);
            sleep(1000);
            drive.shooterServo.setPosition(0);
            sleep(1000);
            drive.shooterServo.setPosition(1);
            sleep(1000);
            drive.shooterServo.setPosition(0);
            sleep(1000);
            drive.shooterServo.setPosition(1);
            sleep(1000);
            drive.shooterServo.setPosition(0);
            sleep(1000);
            drive.followTrajectory(startToB);
            drive.followTrajectory(left);
            drive.releaseGoal();
            sleep(1000);
            drive.deployArm();
            drive.shooter.setPower(0);
            drive.armServo.setPosition(.7);
            drive.followTrajectory(bToGoal);//Go back for second wobble goal
            drive.grabGoal(); //grab wobble goal
            drive.followTrajectory(goalToB);
            drive.releaseGoal();
            sleep(1000);
            drive.followTrajectory(bLine2);
            drive.retractArm();
            sleep(1000);



        }else if (targetZone.equals("A")) {

            drive.grabGoal();
            sleep(500);
            drive.shooter.setVelocity(1600);
            drive.followTrajectory(shooterLine);
            telemetry.update();
            drive.shooterServo.setPosition(1);
            sleep(1000);
            drive.shooterServo.setPosition(0);
            sleep(1000);
            drive.shooterServo.setPosition(1);
            sleep(1000);
            drive.shooterServo.setPosition(0);
            sleep(1000);
            drive.shooterServo.setPosition(1);
            sleep(1000);
            drive.shooterServo.setPosition(0);
            sleep(1000);
            drive.followTrajectory(lineToA);
            drive.releaseGoal();// release goal,
            drive.deployArm();
            sleep(1000);
            drive.shooter.setPower(0);
            drive.armServo.setPosition(.7);
                drive.followTrajectory(aToGoal);           //Go back for second wobble goal
                drive.grabGoal(); //grab wobble goal
            sleep(1000);
            drive.followTrajectory(goalToA);
                drive.releaseGoal();
                sleep(1000);
                drive.followTrajectory(aLine);
            drive.retractArm();
            sleep(1000);




        }
        telemetry.addData("targetVelocity", drive.shooter.getVelocity());
        telemetry.update();
        PoseStorage.currentPose = drive.getPoseEstimate();

    }



    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
            "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
       tfodParameters.minResultConfidence = 0.8f;
       tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
       tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }
}
