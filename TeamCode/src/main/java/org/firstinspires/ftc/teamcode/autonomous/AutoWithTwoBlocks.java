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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.AdrianControls.VuforiaStuff;
import org.firstinspires.ftc.teamcode.drive.MecanumDrive6340;
import org.firstinspires.ftc.teamcode.drive.advanced.PoseStorage;

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
@Autonomous(name = "AutoWithTwoBlocks", group = "Autonomous")
@Disabled

public class AutoWithTwoBlocks extends LinearOpMode {
    private int teamColor;//1=Red -1= Blue

    private ElapsedTime runtime = new ElapsedTime();
    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";
    private String targetZone = "D";
    private static final String VUFORIA_KEY =
            "AZWzerv/////AAABmZeKo4MkD08MoSz5oHB/JU6N1BsUWpfHgQeAeVZemAypSUGVQhvAHo6+v7kJ3MITd8530MhwxRx7GjRtdCs1qjPmdKiJK66dv0yN4Zh4NvKBfP5p4TJjM+G0GoMVgVK0pItm2U56/SVqQH2AYtczQ+giw6zBe4eNhHPJCMY5C2t5Cs6IxxjZlMkRF85l8YAUlKGLipnoZ1T/mX8DNuThQA57qsIB2EN6pGWe8GI64hcPItQ0j7Oyjp82lEN13rYQYsS3Ur4a6//D6yhwa0rogXAysG68G+VgC1mNlj1CjX60qDI84ZN0b/A081xXqjeyFqZK8A/jO8y7BGz9ZuuZNxxXIon6xRNeKYudpfTD23+5";
    private VuforiaLocalizer vuforia;
    public VuforiaStuff vuforiaStuff;
    private TFObjectDetector tfod;

    public AutoWithTwoBlocks(int TeamColor) {
        super();
        teamColor = TeamColor;
    }

    MecanumDrive6340 drive;

    @Override
    public void runOpMode() {
        drive = new MecanumDrive6340(hardwareMap);
        //DetectionLogicForCapstone
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        vuforia = ClassFactory.getInstance().createVuforia(parameters);
        vuforiaStuff = new VuforiaStuff(vuforia);

        Pose2d startPose = new Pose2d(12, -72 * teamColor, Math.toRadians(90 * teamColor));

        drive.setPoseEstimate(startPose);
/*
        Trajectory goForwardFirst = drive.trajectoryBuilder(startPose)
                .splineToConstantHeading(new Vector2d( -36,-71), Math.toRadians(90))
                .build();

        Trajectory goToDuck = drive.trajectoryBuilder(goForwardFirst.end())
                .splineToConstantHeading(new Vector2d( -65,-71), Math.toRadians(90))
                .build();
*/
        Trajectory goToBasketTowerStrafe = drive.trajectoryBuilder(startPose)
                .strafeTo(new Vector2d(-12, -72 * teamColor))
                .build();

//Level Middle
        Trajectory goToBasketTowerLevelMiddle = drive.trajectoryBuilder(startPose)
                .lineTo(new Vector2d(-12, -55 * teamColor))
                .build();
        Trajectory goBackFromBasketTowerLevelMiddle = drive.trajectoryBuilder(goToBasketTowerLevelMiddle.end())
                .lineTo(new Vector2d(-12, -61 * teamColor))
                .build();
        Trajectory GoBackToStartLevelMiddle = drive.trajectoryBuilder(goBackFromBasketTowerLevelMiddle.end())
                //.splineToConst
                // antHeading(new Vector2d( -65,-72), Math.toRadians(90))
                .lineToLinearHeading(new Pose2d(9, -72*teamColor, Math.toRadians(0)))
                .addTemporalMarker(0.5, () ->{

                    //drive.ArmLifter(-1,4);

                })
                .build();

        Trajectory GoToWearhouseLevelMiddle = drive.trajectoryBuilder(GoBackToStartLevelMiddle.end())
                //.splineToConstantHeading(new Vector2d( -65,-72), Math.toRadians(90))
                .lineToLinearHeading(new Pose2d(50, -72*teamColor, Math.toRadians(0)))
                .build();

        Trajectory GoBackToStartLevelMiddleTwo = drive.trajectoryBuilder(GoToWearhouseLevelMiddle.end())
                //.splineToConst
                // antHeading(new Vector2d( -65,-72), Math.toRadians(90))
                .lineToLinearHeading(new Pose2d(9, -72*teamColor, Math.toRadians(0)))
                .build();

        Trajectory goToBasketTowerLevelMiddleTwo = drive.trajectoryBuilder(GoBackToStartLevelMiddleTwo.end())
                .lineToLinearHeading(new Pose2d(-14, -47.5 *teamColor, Math.toRadians(90*teamColor)))
                .build();

        Trajectory GoBackToStartLevelMiddleFinal = drive.trajectoryBuilder(goToBasketTowerLevelMiddleTwo.end())
                //.splineToConst
                // antHeading(new Vector2d( -65,-72), Math.toRadians(90))
                .lineToLinearHeading(new Pose2d(9, -72*teamColor, Math.toRadians(0)))
                .addTemporalMarker(0.5, () ->{

                    //drive.ArmLifter(-1,4);

                })
                .build();
        Trajectory GoToWearhouseLevelMiddleFinal = drive.trajectoryBuilder(GoBackToStartLevelMiddleFinal.end())
                //.splineToConstantHeading(new Vector2d( -65,-72), Math.toRadians(90))
                .lineToLinearHeading(new Pose2d(45, -72*teamColor, Math.toRadians(0)))
                .build();

//Level High
        Trajectory goToBasketTowerLevelHigh = drive.trajectoryBuilder(goToBasketTowerStrafe.end())
                .lineTo(new Vector2d(-12, -47.5 * teamColor))
                .build();
        Trajectory GoBackToStartLevelHigh = drive.trajectoryBuilder(goToBasketTowerLevelHigh.end())
                //.splineToConst
                // antHeading(new Vector2d( -65,-72), Math.toRadians(90))
                .lineToLinearHeading(new Pose2d(12, -72*teamColor, Math.toRadians(0)))
                .addTemporalMarker(0.5, () ->{

                    drive.ArmLifter(-1,4);

                })
                .build();

        Trajectory GoToWearhouseLevelHigh = drive.trajectoryBuilder(GoBackToStartLevelHigh.end())
                //.splineToConstantHeading(new Vector2d( -65,-72), Math.toRadians(90))
                .lineToLinearHeading(new Pose2d(50, -72*teamColor, Math.toRadians(0)))
                .build();

        Trajectory GoBackToStartLevelHighTwo = drive.trajectoryBuilder(GoToWearhouseLevelHigh.end())
                //.splineToConst
                // antHeading(new Vector2d( -65,-72), Math.toRadians(90))
                .lineToLinearHeading(new Pose2d(12, -72*teamColor, Math.toRadians(0)))
                .build();

        Trajectory goToBasketTowerLevelHighTwo = drive.trajectoryBuilder(GoBackToStartLevelHighTwo.end())
                .lineToLinearHeading(new Pose2d(-14, -47.5 * teamColor, Math.toRadians(90*teamColor)))
                .build();

        Trajectory GoBackToStartLevelHighFinal = drive.trajectoryBuilder(goToBasketTowerLevelHighTwo.end())
                //.splineToConst
                // antHeading(new Vector2d( -65,-72), Math.toRadians(90))
                .lineToLinearHeading(new Pose2d(12, -72*teamColor, Math.toRadians(0)))
                .addTemporalMarker(0.5, () ->{

                    drive.ArmLifter(-1,4);

                })
                .build();
        Trajectory GoToWearhouseLevelHighFinal = drive.trajectoryBuilder(GoBackToStartLevelHighFinal.end())
                //.splineToConstantHeading(new Vector2d( -65,-72), Math.toRadians(90))
                .lineToLinearHeading(new Pose2d(45, -72*teamColor, Math.toRadians(0)))
                .build();
        
        
//Level Low
        Trajectory goToBasketTowerLevelLow = drive.trajectoryBuilder(goToBasketTowerStrafe.end())
                .lineTo(new Vector2d(-12, -53.2 * teamColor))
                .build();
/*
        Trajectory gobackfromBasketTowerLevelLow = drive.trajectoryBuilder(goToBasketTowerLevelLow.end())
                .lineTo(new Vector2d(-12, -72 * teamColor))
                .build();
*/
        Trajectory GoBackToStartLevelLow = drive.trajectoryBuilder(goToBasketTowerLevelLow.end())
                //.splineToConst
                // antHeading(new Vector2d( -65,-72), Math.toRadians(90))
                .lineToLinearHeading(new Pose2d(12, -72*teamColor, Math.toRadians(0)))
                .addTemporalMarker(0.5, () ->{

                    drive.ArmLifter(-1,4);

        })
                .build();

        Trajectory GoToWearhouseLevelLow = drive.trajectoryBuilder(GoBackToStartLevelLow.end())
                //.splineToConstantHeading(new Vector2d( -65,-72), Math.toRadians(90))
                .lineToLinearHeading(new Pose2d(50, -72*teamColor, Math.toRadians(0)))
                .build();

        Trajectory GoBackToStartLevelLowTwo = drive.trajectoryBuilder(GoToWearhouseLevelLow.end())
                //.splineToConst
                // antHeading(new Vector2d( -65,-72), Math.toRadians(90))
                .lineToLinearHeading(new Pose2d(12, -72*teamColor, Math.toRadians(0)))
                .build();

        Trajectory goToBasketTowerLevelLowTwo = drive.trajectoryBuilder(GoBackToStartLevelLowTwo.end())
                .lineToLinearHeading(new Pose2d(-14, -47.5 * teamColor, Math.toRadians(90*teamColor)))
                .build();

        Trajectory GoBackToStartLevelLowFinal = drive.trajectoryBuilder(goToBasketTowerLevelLowTwo.end())
                //.splineToConst
                // antHeading(new Vector2d( -65,-72), Math.toRadians(90))
                .lineToLinearHeading(new Pose2d(12, -72*teamColor, Math.toRadians(0)))
                .addTemporalMarker(0.5, () ->{

                    drive.ArmLifter(-1,4);

                })
                .build();
        Trajectory GoToWearhouseLevelLowFinal = drive.trajectoryBuilder(GoBackToStartLevelLowFinal.end())
                //.splineToConstantHeading(new Vector2d( -65,-72), Math.toRadians(90))
                .lineToLinearHeading(new Pose2d(45, -72*teamColor, Math.toRadians(0)))
                .build();



/*
        Trajectory goToWall = drive.trajectoryBuilder(gobackfromBasketTower.end())
                .splineToConstantHeading(new Vector2d( -12,-70*teamColor), Math.toRadians(0))
                .build();

        Trajectory goToShippingHub = drive.trajectoryBuilder(goToWall.end())
                .splineTo(new Vector2d( 36,-71*teamColor), Math.toRadians(0))
                .build();
*/
        /** Wait for the game to begin */
        drive.ArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        telemetry.addData(">", "Ready To Go Teammate. Let's Go ICE 6340!");
        telemetry.update();
        waitForStart();
        drive.ArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //sleep(500);
        drive.ArmLifter(0,4);
        VuforiaStuff.capElementPositionData posData = null;
        posData = vuforiaStuff.vuforiascan(true, true);
        double distanceToDropOffSkystone = 0;
        double distanceBackToCenterLine = 0;
        double distanceBackToSecondStone = 0;
        boolean turnOnlyOneAtIntake = false;
        VuforiaStuff.capElementPos pos = null;
        pos = posData.capElementPosition;


        if (opModeIsActive()) {
            ElapsedTime timeout = new ElapsedTime();
            while (timeout.time() < 2.0) {
                telemetry.addData("Position", pos);
                telemetry.addData("LeftYellowCount", posData.yellowCountLeft);
                telemetry.addData("CenterYellowCount", posData.yellowCountCenter);
                telemetry.addData("RightYellowCount", posData.yellowCountRight);


                telemetry.update();
            }
        }


        //AUTONOMOUS STUFF RIGHT HERE
        targetZone = "D";
        //drive.shooter.setVelocity(1600);
        //   drive.followTrajectory(goForwardFirst);
        //drive.followTrajectory(goToBasketTowerStrafe);
       if(pos == VuforiaStuff.capElementPos.CENTER)
        {
            LiftArmMiddle();
            drive.followTrajectory(goToBasketTowerLevelMiddle);
            drive.outTakeblocks();
            sleep(2000);
            drive.stopIntakeBlocks();
            drive.followTrajectory(goBackFromBasketTowerLevelMiddle);
            drive.ArmLifter(-1,4);
            drive.followTrajectory(GoBackToStartLevelMiddle);
            drive.inTakeblocks();
            drive.followTrajectory(GoToWearhouseLevelMiddle);
            drive.stopIntakeBlocks();
            drive.ArmLifter(1,4);
            drive.followTrajectory(GoBackToStartLevelMiddleTwo);
            drive.ArmLifter(3,4);
            drive.followTrajectory(goToBasketTowerLevelMiddleTwo);
            drive.outTakeblocks();
            sleep(2000);
            drive.stopIntakeBlocks();
            drive.followTrajectory(GoBackToStartLevelMiddleFinal);
            drive.ArmLifter(-1,4);
          //  drive.inTakeblocks();
            drive.followTrajectory(GoToWearhouseLevelMiddleFinal);


        }
        if(pos == VuforiaStuff.capElementPos.RIGHT)
        {
            LiftArmHigh();
            drive.followTrajectory(goToBasketTowerLevelHigh);
            drive.outTakeblocks();
            sleep(2000);
            drive.stopIntakeBlocks();
            drive.followTrajectory(GoBackToStartLevelHigh);
            drive.ArmLifter(-1,4);
            drive.inTakeblocks();
            drive.followTrajectory(GoToWearhouseLevelHigh);
            drive.stopIntakeBlocks();
            drive.ArmLifter(1,4);
            drive.followTrajectory(GoBackToStartLevelHighTwo);
            drive.ArmLifter(3,4);
            drive.followTrajectory(goToBasketTowerLevelHighTwo);
            drive.outTakeblocks();
            sleep(2000);
            drive.stopIntakeBlocks();
            drive.followTrajectory(GoBackToStartLevelHighFinal);
          //  drive.inTakeblocks();
            drive.followTrajectory(GoToWearhouseLevelHighFinal);
        }
        if(pos == VuforiaStuff.capElementPos.LEFT)
        {
            drive.followTrajectory(goToBasketTowerLevelLow);
            drive.outTakeblocks();
            sleep(2000);
            drive.stopIntakeBlocks();
            drive.followTrajectory(GoBackToStartLevelLow);
            drive.ArmLifter(-1,4);
            drive.inTakeblocks();
            drive.followTrajectory(GoToWearhouseLevelLow);
            drive.stopIntakeBlocks();
            drive.ArmLifter(1,4);
            drive.followTrajectory(GoBackToStartLevelLowTwo);
            drive.ArmLifter(3,4);
            drive.followTrajectory(goToBasketTowerLevelLowTwo);
            drive.outTakeblocks();
            sleep(2000);
            drive.stopIntakeBlocks();
            drive.followTrajectory(GoBackToStartLevelLowFinal);
          //  drive.inTakeblocks();
            drive.followTrajectory(GoToWearhouseLevelLowFinal);
        }

//        LiftArm(pos);



        //      drive.followTrajectory(turnToZeroDegree);


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

    private void LiftArmBottom() {
        drive.ArmLifter(0,4);

    }
    private void LiftArmLow() {
        drive.ArmLifter(1,4);

    }

    private void
    LiftArmMiddle() {

        drive.ArmLifter(2,4);
    }

    private void LiftArmHigh() {
        drive.ArmLifter(3,4);


    }

    public void LiftArm(VuforiaStuff.capElementPos pos) {
        if (pos == VuforiaStuff.capElementPos.CENTER) {
            LiftArmMiddle();
        } else if (pos == VuforiaStuff.capElementPos.LEFT) {
            LiftArmLow();
        } else {
            LiftArmHigh();
        }

    }

}
