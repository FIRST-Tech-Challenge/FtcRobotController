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

import static java.lang.Math.abs;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.utility.GamePieceLocation;
import org.firstinspires.ftc.teamcode.vision.util.SpikePosition;

/**
 * Autonomous operation class for 'BlueFieldLeft' scenario.
 * Extends 'AutoBase' which contains code common to all Auto OpModes'.
 */
@Autonomous(name="AutoLineup", group="OpMode")
//@Disabled
public class AutoLineup extends AutoBase {

    int stageNum;

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
    public void init_loop() {
        state = 0;
        stageNum = 0;
        SpikePosition spikePos = getSpikePosition();
        switch (spikePos){
            case LEFT:
                gamepieceLocation = GamePieceLocation.LEFT;
                break;
            case CENTRE:
                gamepieceLocation = GamePieceLocation.CENTER;
                break;
            default:
                gamepieceLocation = GamePieceLocation.RIGHT;
        }

        //telemetry.addData("GamePiece Spike line",gamepieceLocation);
        //telemetry.update();
    }
    // run until the end of the match (driver presses STOP)

    @Override
    public void loop(){
        // Since the Apriltag Yaw is being set to align, its not critical that we start with the IMU at 0 degrees here.
        if (stageNum == 0) {
            moveTo.Rotate(90);
            sleep (1500);
            stageNum = 1;
        } else if (stageNum == 1) {
            if (moveTo.GoToAprilTag(1) == true){
                stageNum = 2;
            };
        }
    }
}
