/* Copyright (c) 2021 FIRST. All rights reserved.
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

package org.firstinspires.ftc.teamcode.opmode.auto;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.utility.AllianceColour;
import org.firstinspires.ftc.teamcode.utility.AprilTagLocation;
import org.firstinspires.ftc.teamcode.utility.GamePieceLocation;
import org.firstinspires.ftc.teamcode.utility.VisionProcessorMode;
import org.firstinspires.ftc.teamcode.vision.util.FieldPosition;
import org.firstinspires.ftc.teamcode.vision.util.SpikePosition;

import java.util.ArrayList;
import java.util.Arrays;

/**
 * Autonomous operation class for 'BlueFieldLeft' scenario.
 * Extends 'AutoBase' which contains code common to all Auto OpModes'.
 */
@Autonomous(name="BlueFieldLeftOdometry", group="OpMode",preselectTeleOp = "GGE Odometry TeleOp")
//@Disabled
public class Auto_BlueFieldLeft_Odometry extends AutoBase {

    /**
     * Runs once and initializes the autonomous program.
     * Sets the initial state and the game piece location to UNDEFINED.
     * If we ever come across an instance in our code where gamepieceLocation is UNDEFINED, there
     * is likely a problem.
     */
    @Override
    public void runOpMode() {
        super.runOpMode();
        gamepieceLocation = GamePieceLocation.UNDEFINED; // this is the position that we can't see
        setFieldPosition(FieldPosition.BLUE_FIELD_LEFT);

        allianceColour = AllianceColour.BLUE;

        // this is setting the initial field coordinates
        // need to set the AprilTagTargets
        targetAprilTags = new ArrayList<>(Arrays.asList(AprilTagLocation.BLUE_LEFT,
                                                        AprilTagLocation.BLUE_CENTRE,
                                                        AprilTagLocation.BLUE_RIGHT));
        lastFieldPos = new Pose2d(0.25,2.2, new Rotation2d(Math.toRadians(0.0)));
        odometry.resetPosition(lastFieldPos,lastFieldPos.getRotation());

        /**
         * This loop is run continuously
         */
        while (opModeInInit()) {
            state = 0;
            SpikePosition spikePos = getSpikePosition();
            switch (spikePos) {
                case LEFT:
                    gamepieceLocation = GamePieceLocation.LEFT;
                    break;
                case CENTRE:
                    gamepieceLocation = GamePieceLocation.CENTER;
                    break;
                default:
                    gamepieceLocation = GamePieceLocation.RIGHT;
            }
            telemetry.addData("GamePiece Spike line", gamepieceLocation);
            telemetry.addData("LSpikeline",getLeftSpikeSaturation());
            telemetry.addData("Cspikeline",getCenterSpikeSaturation());
            telemetry.addData("RSpikeLine",getRightSpikeSaturation());
            telemetry.update();
        }
        while (opModeIsActive()) {
            // we don't want any streaming to the Driver Station, waste of processing and bandwidth
            visionSystem.stopLiveView();

            updateOdometry();

            //we don't need the front camera anymore,  now need the rear one with april tags
            VisionProcessorMode currentVPMode = visionSystem.setVisionProcessingMode(VisionProcessorMode.REAR_CAMERA_BACKDROP_APRIL_TAG);

            // Update IMU and Odometry
            double DirectionNow = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            odometrySpeeds = moveTo.GetWheelSpeeds();
            odometry.updateWithTime(odometryTimer.seconds(),
                    new Rotation2d(Math.toRadians(DirectionNow)), odometrySpeeds);

            // Save our position so TeleOp can pick it up as a starting point
            updateLastPos();

            // Show key stats, useful for the driver and debugging.
            //displayTelemetry(DirectionNow);

            if (gamepieceLocation == GamePieceLocation.LEFT) {
                // Start of Auto when the game piece is on the LEFT spike mark.
                if (state == 0){
                    // Start by securing the loaded pixel  - always pause after servo claw motions.
                    intake.ClawClosed();
                    sleep (250);
                    state = 1;
                } else if (state == 1){
                   if (moveTo.GoToPose2d(new Pose2d(0.4, 2.4, new Rotation2d(Math.toRadians(0.0))))) {
                       state = 2;
                   }
                } else if (state == 2){
                    // Move the claw down
                    intake.FlipDown();
                    state = 3;
                } else if (state == 3) {
                    if (moveTo.GoToPose2d(new Pose2d(0.8, 2.5, new Rotation2d(Math.toRadians(0.0))))) {
                        state = 4;
                    }
                } else if (state == 4) {
                    // Open the claw  - always pause after servo claw motions.
                    intake.ClawOpen();
                    sleep (250);
                    // Move the claw up
                    intake.FlipUp();
                    state = 5;
                } else if (state == 5) {
                    if (moveTo.GoToPose2d(new Pose2d(1.0, 2.85, new Rotation2d(Math.toRadians(-90.0))))) {
                        state = 6;
                    }
                } else if (state == 6) {
                    // Align and drive to April Tag.
                    if (moveTo.GoToAprilTag(AprilTagLocation.BLUE_LEFT)) {
                        state = 7;
                    }
                } else if (state == 7) {
                    //Move the linear slide to the low scoring position
                    linearSlideMove.LinearSlidesLow();
                    // Moves the conveyor forward
                    conveyor.setPosition(0);
                    // Runs the conveyor for 4 seconds
                    sleep(4000);
                    // Stops the conveyor
                    conveyor.setPosition(0.5);
                    state = 8;
                } else if (state == 8) {
                    // Move forward 4 inches (to pre park position)
                    if (moveTo.GoToPose2d(new Pose2d(0.75, 3.0, new Rotation2d(Math.toRadians(-90.0))))) {
                        state = 9;
                    }
                } else if (state == 9) {
                    // Moves the linear slide to the bottom position
                    linearSlideMove.LinearSlidesBottom();
                    // Pause to ensure the lift rest on the bottom
                    sleep(1000);
                    // Finish all autos with the wrist up
                    intake.FlipUp();
                    state = 10;
                } else if (state == 10) {
                    // Moves to park, end auto with state = 99.
                    if (moveTo.GoToPose2d(new Pose2d(0.4, 3.0, new Rotation2d(Math.toRadians(-90.0))))) {
                        state = 99;
                    }
                }
            } else if (gamepieceLocation == GamePieceLocation.CENTER) {
                // Start of Auto when the game piece is on the CENTER spike mark.
                if (state == 0){
                    // Start by securing the loaded pixel  - always pause after servo claw motions.
                    intake.ClawClosed();
                    sleep (250);
                    state = 1;
                } else if (state == 1){
                    // move forward 18 inches
                    if (moveTo.GoToPose2d(new Pose2d(0.6, 2.4, new Rotation2d(Math.toRadians(0.0))))) {
                        state = 2;
                    }
                } else if (state == 2){
                    // Move the claw down
                    intake.FlipDown();
                    state = 3;
                } else if (state == 3){
                    // move forward 4 inches
                    if (moveTo.GoToPose2d(new Pose2d(0.8, 2.4, new Rotation2d(Math.toRadians(0.0))))) {
                        state = 4;
                    }
                } else if (state == 4){
                    // Open the claw  - always pause after servo claw motions.
                    intake.ClawOpen();
                    sleep (250);
                    // Move the claw up
                    intake.FlipUp();
                    state = 5;
                } else if (state == 5){
                    // Rotate 90 degrees
                    if (moveTo.GoToPose2d(new Pose2d(0.8, 2.4, new Rotation2d(Math.toRadians(-90.0))))) {
                        state = 6;
                    }
                } else if (state == 6){
                    // Align and drive to April Tag.  2 is BLUE side CENTER.
                    if (moveTo.GoToAprilTag(AprilTagLocation.BLUE_CENTRE)) {
                        state = 7;
                    }
                } else if (state == 7) {
                    // Move the linear slide to the low scoring position
                    linearSlideMove.LinearSlidesLow();
                    // Moves the conveyor forward
                    conveyor.setPosition(0);
                    // Runs the conveyor for 4 seconds
                    sleep(4000);
                    // Stops the conveyor
                    conveyor.setPosition(0.5);
                    state = 8;
                } else if (state == 8){
                    // Move forward 4 inches (to pre park position)
                    if (moveTo.GoToPose2d(new Pose2d(0.75, 3.0, new Rotation2d(Math.toRadians(-90.0))))) {
                        state = 9;
                    }
                } else if (state == 9) {
                    // Moves the linear slide to the bottom position
                    linearSlideMove.LinearSlidesBottom();
                    // Pause to ensure the lift rest on the bottom
                    sleep(1000);
                    // Finish all autos with the wrist up
                    intake.FlipUp();
                    state = 10;
                } else if (state == 10) {
                    // Moves to park, end auto with state = 99.
                    if (moveTo.GoToPose2d(new Pose2d(0.4, 3.0, new Rotation2d(Math.toRadians(-90.0))))) {
                        state = 99;
                    }
                }
            } else if (gamepieceLocation == GamePieceLocation.RIGHT) {
                // Start of Auto when the game piece is on the RIGHT spike mark.
                if (state == 0){
                    // Start by securing the loaded pixel  - always pause after servo claw motions.
                    intake.ClawClosed();
                    sleep (250);
                    state = 1;
                } else if (state == 1){
                    // move forward 25 inches
                    if (moveTo.GoToPose2d(new Pose2d(0.9, 2.4, new Rotation2d(Math.toRadians(0.0))))) {
                        state = 2;
                    }
                } else if (state == 2){
                    // Rotate 90 degrees
                    if (moveTo.GoToPose2d(new Pose2d(0.9, 2.4, new Rotation2d(Math.toRadians(-90.0))))) {
                        state = 3;
                    }
                } else if (state == 3){
                    // Move the claw down
                    intake.FlipDown();
                    state = 4;
                } else if (state == 4){
                    // move forward 4 inches
                    if (moveTo.GoToPose2d(new Pose2d(0.9, 2.30, new Rotation2d(Math.toRadians(-90.0))))) {
                        state = 5;
                    }
                } else if (state == 5){
                    // Open the claw  - always pause after servo claw motions.
                    intake.ClawOpen();
                    sleep (250);
                    // Move the claw up
                    intake.FlipUp();
                    state = 6;
                } else if (state == 6){
                    // Align and drive to April Tag.  2 is BLUE side CENTER.
                    if (moveTo.GoToAprilTag(AprilTagLocation.BLUE_RIGHT)) {
                        state = 7;
                    }
                } else if (state == 7) {
                    // Move the linear slide to the low scoring position
                    linearSlideMove.LinearSlidesLow();
                    // Moves the conveyor forward
                    conveyor.setPosition(0);
                    // Runs the conveyor for 4 seconds
                    sleep(4000);
                    // Stops the conveyor
                    conveyor.setPosition(0.5);
                    state = 8;
                } else if (state == 8){
                    // Move forward 4 inches (to pre park position)
                    if (moveTo.GoToPose2d(new Pose2d(0.5, 2.9, new Rotation2d(Math.toRadians(-90.0))))) {
                        state = 9;
                    }
                } else if (state == 9) {
                    // Moves the linear slide to the bottom position
                    linearSlideMove.LinearSlidesBottom();
                    state = 10;
                } else if (state == 10) {
                    // Moves to park, end auto with state = 99.
                    if (moveTo.GoToPose2d(new Pose2d(0.2, 3.2, new Rotation2d(Math.toRadians(-90.0))))) {
                        state = 99;
                    }
                }
            }
        }
    }
}
