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

package org.firstinspires.ftc.teamcode.opmodes.auto.reference;

// RR-specific imports
import static com.qualcomm.robotcore.util.ElapsedTime.Resolution.SECONDS;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;

// Non-RR imports
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.roadrunner.teamcode.MecanumDrive;

/**
 * FTC WIRES Autonomous Example for only vision detection using tensorflow and park
 */
@Autonomous(name = "FTC Wires IntoTheDeep", group = "00-Autonomous", preselectTeleOp = "FTC Wires TeleOp")
@Disabled
public class FTCWIRES_AUTO extends LinearOpMode {

    public static String TEAM_NAME = "Pack-A-Punch"; //TODO: Enter team Name
    public static int TEAM_NUMBER = 26396; //TODO: Enter team Number

    //Define and declare Robot Starting Locations
    public enum START_POSITION{
        LEFT,
        RIGHT
    }
    public static START_POSITION startPosition;

    @Override
    public void runOpMode(){

        //Key Pad input to selecting Starting Position of robot
        telemetry.setAutoClear(true);
        telemetry.clearAll();
        while(!isStopRequested()){
            telemetry.addData("Initializing FTC Wires (ftcwires.org) Autonomous adopted for Team:",
                    TEAM_NAME, " ", TEAM_NUMBER);
            telemetry.addData("---------------------------------------","");
            telemetry.addData("Select Starting Position using XYAB on Logitech (or ▢ΔOX on Playstayion) on gamepad 1:","");
            telemetry.addData("    Left   ", "(X / ▢)");
            telemetry.addData("    Right ", "(Y / Δ)");

            if(gamepad1.x){
                startPosition = START_POSITION.LEFT;
                break;
            }
            if(gamepad1.y){
                startPosition = START_POSITION.RIGHT;
                break;
            }
            telemetry.update();
        }
        telemetry.setAutoClear(false);
        telemetry.clearAll();

        telemetry.addData("Selected Starting Position", startPosition);
        telemetry.update();

        waitForStart();

        //Game Play Button  is pressed
        if (opModeIsActive() && !isStopRequested()) {
            //Build parking trajectory based on last detected target by vision
            runAutonoumousMode();
        }
    }   // end runOpMode()

    public void runAutonoumousMode() {
        //Initialize Pose2d as desired
        Pose2d initPose = new Pose2d(0, 0, Math.toRadians(0)); // Starting Pose
        Pose2d submersibleSpecimen = new Pose2d(1,0,Math.toRadians(0) );
        Pose2d netZone = new Pose2d(2,0,Math.toRadians(0));
        Pose2d yellowSampleOne = new Pose2d(3,0,Math.toRadians(0));
        Pose2d yellowSampleTwo = new Pose2d(4,0,Math.toRadians(0));
        Pose2d submersiblePark = new Pose2d(5,0,Math.toRadians(0));


        Pose2d observationZone = new Pose2d(6,0,Math.toRadians(0));
        Pose2d specimenPickup = new Pose2d(7,0,Math.toRadians(0));
        Pose2d colorSampleOne = new Pose2d(8,0,Math.toRadians(0));
        Pose2d observationPark = new Pose2d(9,0,Math.toRadians(0));

        double waitSecondsBeforeDrop = 0;
        MecanumDrive drive = new MecanumDrive(hardwareMap, initPose);

        if (startPosition == START_POSITION.LEFT) {

            //Move robot to submersible to place specimen
            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .strafeToLinearHeading(submersibleSpecimen.position, submersibleSpecimen.heading)
                            .build());
            safeWaitSeconds(1);
            telemetry.addLine("Move robot to submersible to place specimen on chamber");
            telemetry.update();

            // Add code for placing specimen on chamber and pick sample from inside submersible
            safeWaitSeconds(1);
            telemetry.addLine("Place specimen on chamber and pick sample from inside submersible");
            telemetry.update();

            //Move robot to netZone
            Actions.runBlocking(
                    drive.actionBuilder(submersibleSpecimen)
                            .strafeToLinearHeading(netZone.position, netZone.heading)
                            .build());
            safeWaitSeconds(1);
            telemetry.addLine("Move robot to netZone");
            telemetry.update();

            //Add code to drop sample in basket
            safeWaitSeconds(1);
            telemetry.addLine("Drop sample in basket");
            telemetry.update();

            //Move robot to pick yellow sample one
            Actions.runBlocking(
                    drive.actionBuilder(netZone)
                            .strafeToLinearHeading(yellowSampleOne.position, yellowSampleOne.heading)
                            .build());
            safeWaitSeconds(1);
            telemetry.addLine("Move robot to pick yellow sample one");
            telemetry.update();

            //Add code to pick up yellow sample
            safeWaitSeconds(1);
            telemetry.addLine("Pick up yellow sample");
            telemetry.update();

            //Move robot to net zone to drop sample
            Actions.runBlocking(
                    drive.actionBuilder(yellowSampleOne)
                            .strafeToLinearHeading(netZone.position, netZone.heading)
                            .build());
            safeWaitSeconds(1);
            telemetry.addLine("Move robot to net zone to drop sample");
            telemetry.update();

            //Add code to drop sample in bucket
            safeWaitSeconds(1);
            telemetry.addLine("Drop sample in bucket");
            telemetry.update();

            //Move robot to yellow sample two
            Actions.runBlocking(
                    drive.actionBuilder(netZone)
                            .strafeToLinearHeading(yellowSampleTwo.position, yellowSampleTwo.heading)
                            .build());
            safeWaitSeconds(1);
            telemetry.addLine("Move robot to yellow sample two");
            telemetry.update();

            //Add code to pick up yellow sample
            safeWaitSeconds(1);
            telemetry.addLine("Pick up yellow sample");
            telemetry.update();

            //Move robot to net zone
            Actions.runBlocking(
                    drive.actionBuilder(yellowSampleTwo)
                            .strafeToLinearHeading(netZone.position, netZone.heading)
                            .build());
            safeWaitSeconds(1);
            telemetry.addLine("Move robot to net zone");
            telemetry.update();

            //Add code to drop sample in bucket
            safeWaitSeconds(1);
            telemetry.addLine("Drop sample in bucket");
            telemetry.update();

            //Move robot to submersible parking
            Actions.runBlocking(
                    drive.actionBuilder(netZone)
                            .strafeToLinearHeading(submersiblePark.position, submersiblePark.heading)
                            .build());
            safeWaitSeconds(1);
            telemetry.addLine("Move robot to submersible parking");
            telemetry.update();
        } else {

            //Move robot to submersible to place specimen
            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .strafeToLinearHeading(submersibleSpecimen.position, submersibleSpecimen.heading)
                            .build());
            safeWaitSeconds(1);
            telemetry.addLine("Move robot to submersible to place specimen");
            telemetry.update();

            //Add code to place specimen on chamber and pick up sample
            safeWaitSeconds(1);
            telemetry.addLine("Place specimen on chamber and pick up sample");
            telemetry.update();

            //Move robot to observation zone
            Actions.runBlocking(
                    drive.actionBuilder(submersibleSpecimen)
                            .strafeToLinearHeading(observationZone.position, observationZone.heading)
                            .build());
            safeWaitSeconds(1);
            telemetry.addLine("Move robot to observation zone");
            telemetry.update();

            //Add code to drop sample in observation zone
            safeWaitSeconds(1);
            telemetry.addLine("Drop sample in observation zone");
            telemetry.update();

            //Move robot to color sample 1
            Actions.runBlocking(
                    drive.actionBuilder(observationZone)
                            .strafeToLinearHeading(colorSampleOne.position, colorSampleOne.heading)
                            .build());
            safeWaitSeconds(1);
            telemetry.addLine("Move robot to color sample 1");
            telemetry.update();

            //Add code to pick up color sample 1
            safeWaitSeconds(1);
            telemetry.addLine("Pick up color sample 1");
            telemetry.update();

            //Move robot to observation zone
            Actions.runBlocking(
                    drive.actionBuilder(colorSampleOne)
                            .strafeToLinearHeading(observationZone.position, observationZone.heading)
                            .build());
            safeWaitSeconds(1);
            telemetry.addLine("Move robot to observation zone");
            telemetry.update();

            //Add code to drop observation zone
            safeWaitSeconds(1);
            telemetry.addLine("Drop observation zone");
            telemetry.update();

            //Move robot to pickup specimen
            Actions.runBlocking(
                    drive.actionBuilder(observationZone)
                            .strafeToLinearHeading(specimenPickup.position, specimenPickup.heading)
                            .build());
            safeWaitSeconds(1);
            telemetry.addLine("Move robot to pickup specimen");
            telemetry.update();

            //Add code to pick up specimen
            safeWaitSeconds(1);
            telemetry.addLine("Pick up specimen");
            telemetry.update();

            //Move robot to submersible
            Actions.runBlocking(
                    drive.actionBuilder(specimenPickup)
                            .strafeToLinearHeading(submersibleSpecimen.position, submersibleSpecimen.heading)
                            .build());
            safeWaitSeconds(1);
            telemetry.addLine("Move robot to submersible");
            telemetry.update();

            //Add code to place sample on chamber and pickup sample
            safeWaitSeconds(1);
            telemetry.addLine("place sample on chamber and pickup sample");
            telemetry.update();

            //Move robot to observation parking
            Actions.runBlocking(
                    drive.actionBuilder(submersibleSpecimen)
                            .strafeToLinearHeading(observationPark.position, observationPark.heading)
                            .build());
            safeWaitSeconds(1);
            telemetry.addLine("Move robot to observation parking");
            telemetry.update();
        }

    }

    //method to wait safely with stop button working if needed. Use this instead of sleep
    public void safeWaitSeconds(double time) {
        ElapsedTime timer = new ElapsedTime(SECONDS);
        timer.reset();
        while (!isStopRequested() && timer.time() < time) {
        }
    }
}   // end class
