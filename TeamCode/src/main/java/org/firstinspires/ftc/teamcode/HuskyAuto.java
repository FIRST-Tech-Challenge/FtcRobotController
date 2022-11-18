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

import static org.firstinspires.ftc.teamcode.HuskyBot.CLAW_GRAB_OPEN_POSITION;
import static org.firstinspires.ftc.teamcode.HuskyBot.CLAW_LIFT_START_POSITION;
import static org.firstinspires.ftc.teamcode.HuskyBot.CLAW_ROTATE_START_POSITION;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.openftc.apriltag.AprilTagDetection;
import java.util.ArrayList;

@Autonomous(name = "Auto", group = "Auto", preselectTeleOp = "Husky TeleOpMode")
public class HuskyAuto extends HuskyAutoBase {
    boolean tagFound = false;

    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();

        waitForStart();
        runtime.reset();

        // initial wait time if needed
        while (opModeIsActive() && (runtime.seconds() < INIT_WAIT_SECS)) {
        }

        telemetry.addLine("Searching for the tag..");
        telemetry.update();

        runtime.reset();

        // get the park location with timeout of 5 seconds.
        while (opModeIsActive() && (runtime.seconds() < 5)) {
            ArrayList<AprilTagDetection> currentDetections = pipeline.getLatestDetections();

            if(currentDetections.size() != 0) {
                for(AprilTagDetection tag : currentDetections) {
                    switch (tag.id){
                        case LOCATION_1_TAG_ID:
                            this.tagFound = true;
                            this.parkLocation = Location.LOCATION_1;
                            break;
                        case LOCATION_2_TAG_ID:
                            this.tagFound = true;
                            this.parkLocation = Location.LOCATION_2;
                            break;
                        case LOCATION_3_TAG_ID:
                            this.tagFound = true;
                            this.parkLocation = Location.LOCATION_3;
                            break;
                    }
                }
            }

            sleep(50);
        }

        if(this.tagFound)
            telemetry.addLine("Found the tag! Parking at location " + this.parkLocation);
        else{
            // Set the park location to 2 if it couldn't detect the apriltag.
            this.parkLocation = Location.LOCATION_2;
            telemetry.addLine("Couldn't find the tag! Parking at location 2 (33.3% chance :D) " + this.parkLocation);
        }

        telemetry.update();

        // Open the claw before moving forward
        huskyBot.clawLift.setPosition(CLAW_LIFT_START_POSITION);
        huskyBot.clawGrab.setPosition(CLAW_GRAB_OPEN_POSITION);
        huskyBot.clawRotate.setPosition(CLAW_ROTATE_START_POSITION);

        if (this.parkLocation == Location.LOCATION_1) {
            // If the park location is 1, park at location 1
            encoderDrive(AUTO_DRIVE_SPEED, FORWARD_DISTANCE, 2);
            encoderStrafe(AUTO_STRAFE_SPEED, -STRAFE_DISTANCE, 2);
        } else if (this.parkLocation == Location.LOCATION_2) {
            // If the park location is 2, park at location 2
            encoderDrive(AUTO_DRIVE_SPEED, FORWARD_DISTANCE, 2);
        } else if(this.parkLocation == Location.LOCATION_3) {
            // If the park location is 1, park at location 3
            encoderDrive(AUTO_DRIVE_SPEED, FORWARD_DISTANCE, 2);
            encoderStrafe(AUTO_STRAFE_SPEED, STRAFE_DISTANCE, 2);
        }

        telemetry.addData("Path", "Complete");
        telemetry.update();
    }
}