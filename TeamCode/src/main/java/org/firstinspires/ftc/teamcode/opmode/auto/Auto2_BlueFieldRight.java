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

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import org.firstinspires.ftc.teamcode.utility.GamePieceLocation;

import org.firstinspires.ftc.teamcode.vision.util.SpikePosition;

/**
 * Autonomous operation class for 'BlueFieldLeft' scenario.
 * Extends 'AutoBase' which contains code common to all Auto OpModes'.
 */

@Autonomous(name="BlueFieldRight", group="OpMode")
//@Disabled
public class Auto2_BlueFieldRight extends AutoBase {

    /**
     * Runs once and initializes the autonomous program.
     * Sets the initial state and the game piece location to UNDEFINED.
     * If we ever come across an instance in our code where gamepieceLocation is UNDEFINED, there
     * is likely a problem.
     */
    @Override
    public void init() {
        super.init();
        gamepieceLocation = GamePieceLocation.UNDEFINED; // this is the position that we can't see
    }

    /**
     * This loop is run continuously
     */
    @Override
    public void init_loop(){
        state = 0;
        SpikePosition spikePos = getSpikePosition();
        switch (spikePos){
            case RIGHT:
                gamepieceLocation = GamePieceLocation.RIGHT;
                break;
            case CENTRE:
                gamepieceLocation = GamePieceLocation.CENTER;
                break;
            default:
                gamepieceLocation = GamePieceLocation.LEFT;
        }
        telemetry.addData("GamePiece Spike line",gamepieceLocation);
        telemetry.update();
    }

    @Override
    public void loop(){

        //moveTo.Forward(200);

        double DirectionNow = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

        if (gamepieceLocation == GamePieceLocation.RIGHT && state == 0){
            // Start by securing the loaded pixel
            intake.ClawClosed();
            // move forward 2 inches
            moveTo.Forward((int)((2 * ticksPerInch) * 0.94), 0.25); // Calculated ticks by distance * 94% (from last year)
            // move sideways 9 inches
            moveTo.Right((int)((9 * ticksPerInch)* 1.04), 0.5); // Calculated ticks by distance * 104% (from last year)
            // move forward 12 inches
            moveTo.Forward((int)((12 * ticksPerInch) * 0.94), 0.25); // Calculated ticks by distance * 94% (from last year)
            // Move the claw down
            intake.FlipDown();
            sleep (500);
            // Open the claw
            intake.ClawOpen();
            // End all autos with the wrist up
            intake.FlipUp();
            // Add telemetry
            telemetry.addData("run", state);
            telemetry.update();
            state = 1;
        } else if (gamepieceLocation == GamePieceLocation.CENTER && state == 0) {
            // Start by securing the loaded pixel
            intake.ClawClosed();
            // move forward 18 inches
            moveTo.Forward((int)((18 * ticksPerInch) * 0.94), 0.25); // Calculated ticks by distance * 94% (from last year)
            // Move the claw down
            intake.FlipDown();
            sleep (500);
            // Move forward 4 inches
            moveTo.Forward((int)((4 * ticksPerInch) * 0.94), 0.25);
            // Open the claw
            intake.ClawOpen();
            // End all autos with the wrist up
            intake.FlipUp();
            state = 2;
        } else if (state == 0) {
            // Start by securing the loaded pixel
            intake.ClawClosed();
            // Move forward 25 inches
            moveTo.Forward((int)((25 * ticksPerInch) * 0.94), 0.25);
            // Rotate 90 degrees
            moveTo.Rotate(-95);
            sleep(700);
            // Move the claw down
            intake.FlipDown();
            sleep(1500);
            // Move forward 6 inches
            moveTo.Forward((int)((5.5 * ticksPerInch) * 0.94), 0.4);
            // Open the claw
            intake.ClawOpen();
            sleep(500);
            // End all autos with the wrist up
            intake.FlipUp();
            state = 3;
        }

        // Show the elapsed game time and wheel power.
        displayTelemetry(DirectionNow);
    }}
