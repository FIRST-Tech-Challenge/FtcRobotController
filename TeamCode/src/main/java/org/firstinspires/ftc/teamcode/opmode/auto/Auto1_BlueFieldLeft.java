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

import static android.os.SystemClock.sleep;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.checkerframework.checker.units.qual.C;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import org.firstinspires.ftc.teamcode.utility.GamePieceLocation;

import org.firstinspires.ftc.teamcode.vision.util.FieldPosition;
import org.firstinspires.ftc.teamcode.vision.util.SpikePosition;
import org.firstinspires.ftc.vision.VisionPortal;

/**
 * Autonomous operation class for 'BlueFieldLeft' scenario.
 * Extends 'AutoBase' which contains code common to all Auto OpModes'.
 */
@Autonomous(name="BlueFieldLeft", group="OpMode",preselectTeleOp = "GGE Drive T2")
//@Disabled
public class Auto1_BlueFieldLeft extends AutoBase {

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


        /**
         * This loop is run continuously
         */
        while (opModeInInit()) {

            // put this if in to address an error I remembered from pre tournament.  may not need this
            if (!isStopRequested() && (myVisionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                sleep(20);
            }
            else {
                // To start auto, we will be using the front camera
                if (rearCam == myVisionPortal.getActiveCamera()) {
                    myVisionPortal.setActiveCamera(frontCam);
                }
            }

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
            telemetry.addData("Camera state",myVisionPortal.getCameraState());
            telemetry.update();
        }
        while (opModeIsActive()) {

            double DirectionNow = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

            if (gamepieceLocation == GamePieceLocation.LEFT && state == 0) {
                // Start by securing the loaded pixel  - always pause after servo claw motions.
                intake.ClawClosed();
                sleep (250);
                // move forward 2 inches
                moveTo.Forward((int) ((2 * ticksPerInch) * 0.94), 0.25); // Calculated ticks by distance * 94% (from last year)
                // move sideways 9 inches
                moveTo.Left((int) ((9 * ticksPerInch) * 1.04), 0.4); // Calculated ticks by distance * 104% (from last year)
                // move forward 8 inches
                moveTo.Forward((int) ((8 * ticksPerInch) * 0.94), 0.25); // Calculated ticks by distance * 94% (from last year)
                sleep (500);
                // Move the claw down
                intake.FlipDown();
                sleep (500);
                // move forward 6 inches
                moveTo.Forward((int) ((6 * ticksPerInch) * 0.94), 0.25); // Calculated ticks by distance * 94% (from last year)
                sleep (500);
                // Open the claw  - always pause after servo claw motions.
                intake.ClawOpen();
                sleep (250);
                // Move the claw up
                intake.FlipUp();
                // Rotate 90 degrees
                moveTo.Rotate(90);
                sleep(700);
                state = 1;
            } else if (gamepieceLocation == GamePieceLocation.CENTER && state == 0) {
                // Start by securing the loaded pixel - always pause after servo claw motions.
                intake.ClawClosed();
                sleep (250);
                // move forward 18 inches
                moveTo.Forward((int) ((18 * ticksPerInch) * 0.94), 0.25); // Calculated ticks by distance * 94% (from last year)
                // Move the claw down
                intake.FlipDown();
                // Move forward 4 inches
                moveTo.Forward((int) ((4 * ticksPerInch) * 0.94), 0.25);
                // Open the claw  - always pause after servo claw motions.
                intake.ClawOpen();
                sleep (250);
                // Move the claw up
                intake.FlipUp();
                // Rotate 90 degrees
                moveTo.Rotate(90);
                sleep(700);
                state = 1;
            } else if (gamepieceLocation == GamePieceLocation.RIGHT && state == 0) {
                // Start by securing the loaded pixel - always pause after servo claw motions.
                intake.ClawClosed();
                sleep (250);
                moveTo.Forward((int) ((25 * ticksPerInch) * 0.94), 0.25);
                moveTo.Rotate(90);
                sleep(700);
                intake.FlipDown();
                moveTo.Forward((int) ((4 * ticksPerInch) * 0.94), 0.25);
                // Deposit the preloaded claw - always pause after servo claw motions.
                intake.ClawOpen();
                sleep(250);
                intake.FlipUp();
                state = 1;
            }

            // Use the GoToAprilTag to get to within 7 inches of the Backdrop
            if (gamepieceLocation == GamePieceLocation.LEFT && state == 1) {
                // Align and drive to April Tag.  1 is BLUE side LEFT.
                if (moveTo.GoToAprilTag(1) == true) {
                    state = 2;
                }
            } else if (gamepieceLocation == GamePieceLocation.CENTER && state == 1) {
                // Align and drive to April Tag.  2 is BLUE side CENTER.
                if (moveTo.GoToAprilTag(2) == true) {
                    state = 2;
                }
            } else if (gamepieceLocation == GamePieceLocation.RIGHT && state == 1) {
                // Align and drive to April Tag.  3 is BLUE side RIGHT.
                if (moveTo.GoToAprilTag(3) == true) {
                    state = 2;
                }
            }

            // Complete the auto by dropping the game piece and going to park
            if (gamepieceLocation == GamePieceLocation.LEFT && state == 2) {
                //Move back 5 more inches after the April Tag positioning completes.
                moveTo.Backwards((int) ((5 * ticksPerInch) * 0.94), 0.25);
                //Move the linear slide to the low scoring position
                linearSlideMove.LinearSlidesLow();
                // Moves the conveyor forward
                conveyor.setPosition(0);
                // Runs the conveyor for 4 seconds
                sleep(4000);
                // Stops the conveyor
                conveyor.setPosition(0.5);
                // Moves the linear slide to the bottom position
                linearSlideMove.LinearSlidesBottom();
                // Pause to ensure the lift rest on the bottom
                sleep(500);
                // Finish all autos with the wrist up
                intake.FlipUp();
                // Forward 4 inches
                moveTo.Forward((int) ((4 * ticksPerInch) * 0.94), 0.25);
                // Moves right 18 inches
                moveTo.Right((int) ((18 * ticksPerInch) * 1.04), 0.4);
                // Backward 6 inches
                moveTo.Backwards((int) ((10 * ticksPerInch) * 0.94), 0.25);
                // Add telemetry
                telemetry.addData("run", state);
                telemetry.update();
                state = 3;
            } else if (gamepieceLocation == GamePieceLocation.CENTER && state == 2) {
                //Move back 5 more inches after the April Tag positioning completes.
                moveTo.Backwards((int) ((5 * ticksPerInch) * 0.94), 0.25);
                // Move the linear slide to the low scoring position
                linearSlideMove.LinearSlidesLow();
                // Moves the conveyor forward
                conveyor.setPosition(0);
                // Runs the conveyor for 4 seconds
                sleep(4000);
                // Stops the conveyor
                conveyor.setPosition(0.5);
                // Moves the linear slide to the bottom position
                linearSlideMove.LinearSlidesBottom();
                // Pause to ensure the lift rest on the bottom
                sleep(500);
                // Finish all autos with the wrist up
                intake.FlipUp();
                // Forward 4 inches
                moveTo.Forward((int) ((4 * ticksPerInch) * 0.94), 0.25);
                // Moves right 26 inches
                moveTo.Right((int) ((24 * ticksPerInch) * 1.04), 0.4);
                // Backward 6 inches
                moveTo.Backwards((int) ((10 * ticksPerInch) * 0.94), 0.25);
                state = 3;
            } else if (gamepieceLocation == GamePieceLocation.RIGHT && state == 2) {
                // Move back 5 more inches after the April Tag positioning completes.
                moveTo.Backwards((int) ((5 * ticksPerInch) * 0.94), 0.25);
                // Move the linear slide to the low scoring position
                linearSlideMove.LinearSlidesLow();
                // Moves the conveyor forward
                conveyor.setPosition(0);
                // Runs the conveyor for 4 seconds
                sleep(4000);
                // Stops the conveyor
                conveyor.setPosition(0.5);
                // Moves the linear slide to the bottom position
                linearSlideMove.LinearSlidesBottom();
                // Pause to ensure the lift rest on the bottom
                sleep(500);
                // Finish all autos with the wrist up
                intake.FlipUp();
                // Forward 4 inches
                moveTo.Forward((int) ((4 * ticksPerInch) * 0.94), 0.25);
                // Moves right 32 inches
                moveTo.Right((int) ((32 * ticksPerInch) * 1.04), 0.4);
                // Backward 10 inches
                moveTo.Backwards((int) ((10 * ticksPerInch) * 0.94), 0.25);
                state = 3;
            }
            // Show the elapsed game time and wheel power.
            displayTelemetry(DirectionNow);
        }

        // this should happen automagically ... should
        // we were crashing at the end of auto perhaps this will help
        if(isStopRequested()){
            myVisionPortal.close();
        }
    }
}
