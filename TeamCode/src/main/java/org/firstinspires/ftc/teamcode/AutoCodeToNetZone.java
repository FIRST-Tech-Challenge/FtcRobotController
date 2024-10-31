/* Copyright (c) 2017 FIRST. All rights reserved.
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

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name="Auto Place Specimen Pull Sample to Net", group="")
public class AutoCodeToNetZone extends LinearOpMode {

    static final double     DRIVE_SPEED             = 0.3;
    static final double     DRIVE_INCREASED_SPEED             = 0.8;
    static final double     TURN_SPEED              = 0.2;

    private HornetRobo hornetRobo;

    //manager classes
    private AutoDriveManager driveManager;
    private AutoArmManager armManager;
    private AutoGrabberManager grabberManager;
    private AutoViperSlideManager viperSlideManager;

    //path flags
    private boolean strafeToStart = true;
    private boolean dropSpecimen = true;
    private boolean moveNearToAscentZone = true;
    private boolean ploughSampleToNetZone = true;
    private boolean parkInAscent = true;

    public void initialize()
    {
        hornetRobo = new HornetRobo();
        AutoHardwareMapper.MapToHardware(this, hornetRobo);
        driveManager = new AutoDriveManager(this, hornetRobo);
        grabberManager = new AutoGrabberManager(this, hornetRobo);
        armManager = new AutoArmManager(this, hornetRobo);
        viperSlideManager = new AutoViperSlideManager(this, hornetRobo);
    }

    public void runOpMode() {
        initialize();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        if (opModeIsActive()) {
            telemetry.addData("Starting to move", "");
            telemetry.update();

            //set forward
            driveManager.SetMotorDirection(AutoDriveManager.DriveDirection.FORWARD);
            viperSlideManager.SetDirection(AutoDriveManager.DriveDirection.FORWARD);
            while (opModeIsActive() && !isStopRequested()) {

                telemetry.addData("Move to reach submersible  ", "");
                telemetry.update();
                int strafeDistance;
                int distanceToSub;
                int distance;
                int rotateToPosition;

                if (strafeToStart) {

                    telemetry.addData("Strafe to align with submersible", "");
                    telemetry.update();

                    strafeDistance = 37;
                    driveManager.StrafeToPosition(AutoDriveManager.DriveDirection.RIGHT, DRIVE_SPEED, strafeDistance);
                    telemetry.addData("Strafed Right", "");
                    telemetry.update();
                }

                if (dropSpecimen) {

                    //TODO: adjust the distance during testing
                    distanceToSub = 27;
                    telemetry.addData("Distance To Sub: ", distanceToSub);
                    telemetry.update();

                    //set target position to encoders and move to position
                    driveManager.MoveStraightToPosition(AutoDriveManager.DriveDirection.BACKWARD, DRIVE_SPEED, distanceToSub);
                    telemetry.addData("Reached Submersible", "");
                    telemetry.update();

                    //Set Arm in a position to hang specimen
                    double armPosition = 0.4; //TODO: Correct during testing
                    armManager.MoveArmToPosition(armPosition);
                    telemetry.addData("Set Arm Pos: ", armPosition);
                    telemetry.update();

                    int reverseDistance = -6; //TODO: Adjust during testing
                    driveManager.MoveStraightToPosition(AutoDriveManager.DriveDirection.FORWARD, DRIVE_SPEED + 0.2, reverseDistance);
                    telemetry.addData("Go reverse to be away from submersible to move to pick sample", reverseDistance);
                    telemetry.update();

                    //Move arm back
                    armPosition = 0.6; //TODO: Correct during testing
                    armManager.MoveArmToPosition(armPosition);
                    telemetry.addData("Set Arm Pos Back: ", armPosition);
                    telemetry.update();
                    sleep(100);
                }

                if (moveNearToAscentZone) {
                    // Go reverse to be away from submersible to move to pick sample
                    int reverseDistance = 5; //TODO: Adjust during testing
                    driveManager.MoveStraightToPosition(AutoDriveManager.DriveDirection.FORWARD, DRIVE_SPEED, reverseDistance);
                    telemetry.addData("Go reverse to be away from submersible to move to pick sample", reverseDistance);
                    telemetry.update();

                    //strafe to go pass submersible edges to avoid hitting when moving forward to pick samples
                    strafeDistance = 30; //TODO: Adjust during testing
                    driveManager.StrafeToPosition(AutoDriveManager.DriveDirection.LEFT, DRIVE_SPEED, strafeDistance);
                    telemetry.addData("strafe to go pass submersible edges to avoid hitting when moving forward to pick samples", strafeDistance);
                    telemetry.update();

                    //MOVE FORWARD past sample
                    distance = 30; //TODO: Adjust during testing
                    driveManager.MoveStraightToPosition(AutoDriveManager.DriveDirection.BACKWARD, DRIVE_SPEED, distance);
                    telemetry.addData("MOVE FORWARD past sample", distance);
                    telemetry.update();
                }

                if (ploughSampleToNetZone) {

                    //Rotate 180 (to point front) + 45 degrees to turn to go in diagonal to plogh sample
                    rotateToPosition = 30;
                    driveManager.TurnUsingEncoders(AutoDriveManager.DriveDirection.LEFT, DRIVE_SPEED, rotateToPosition);
                    telemetry.addData("rotate to point to front and turn to plough the sample", "");
                    telemetry.update();

                    //Move forward until it reaches net zone
                    distance = 35; //TODO: Adjust during testing
                    driveManager.MoveStraightToPosition(AutoDriveManager.DriveDirection.BACKWARD, DRIVE_SPEED, distance);
                    telemetry.addData("Move backward until it reaches net zone", "");
                    telemetry.update();

                    rotateToPosition = 30;
                    driveManager.TurnUsingEncoders(AutoDriveManager.DriveDirection.LEFT, DRIVE_SPEED, rotateToPosition);
                    telemetry.addData("rotate to point to front and turn to plough the sample", "");
                    telemetry.update();
                }

                if (parkInAscent)
                {
                    distance = 35;
                    driveManager.MoveStraightToPosition(AutoDriveManager.DriveDirection.FORWARD, DRIVE_SPEED, distance);
                }

                //done
                driveManager.StopRobo();
                telemetry.addData("Stopped Robo", "");
                telemetry.update();

                break;

            }
        }
    }
}