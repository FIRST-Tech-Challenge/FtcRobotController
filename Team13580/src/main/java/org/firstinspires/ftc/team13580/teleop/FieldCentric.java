/* Copyright (c) 2022 FIRST. All rights reserved.
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

package org.firstinspires.ftc.team13580.teleop; // TODO(STUDENTS): Change to your team package (e.g., org.firstinspires.ftc.team12345.teleop)

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.team13580.RobotHardware;

@TeleOp(name="Field Centric", group="TeleOp")
// TODO(STUDENTS): You may rename this for your robot (e.g., "Field Centric - Comp Bot)
public class FieldCentric extends LinearOpMode {

    // NOTE: One hardware instance per OpMode keeps mapping/IMU use simple and testable
    RobotHardware robot = new RobotHardware(this);

    @Override
    public void runOpMode() {
        // Driver inputs (range roughly [-1, 1])
        double axial    = 0; // forward/back (+ forward)
        double lateral  = 0; // strafe left/right (+ right)
        double yaw      = 0; // rotation (+ CCW/left turn)

        // --- INIT PHASE ---
        // WHY: Centralized init in RobotHardware sets motor directions, encoder modes, IMU orientation, etc.
        // TODO(STUDENTS): Confirm IMU orientation & Motor names in RobotHardware.init()
        robot.init();

        // Wait for driver to press START on Driver Station
        waitForStart();

        // --- TELEOP LOOP ---
        while (opModeIsActive()) {

            // READ sticks
            // NOTE: FTC gamepads support negative when pushing left_stick_y forward, so we invert it.
            axial   = -gamepad1.left_stick_y;
            lateral =  gamepad1.left_stick_x;
            yaw     =  gamepad1.right_stick_x;

            // Optional: Deadband to ignore tiny stick noise (uncomment to use)
            // double dead = 0.05: // TODO(STUDENTS): tune
            // axial   = (Math.abs(axial)   < dead) ? 0 : axial;
            // lateral = (Math.abs(lateral) < dead) ? 0 : lateral;
            // yaw     = (Math.abs(yaw)     < dead) ? 0 : yaw;

            // Optional: Precision/slow mode (hold Left Trigger to reduce sensitivity)
            // double slow = 1.0 - (0.6 * gamepad1.left_trigger); // 1.0 → 0.4
            // axial *= slow; lateral *= slow; yaw *= slow;

            // Optional: Zero heading on button (keeps field-forward aligned to current front)
            // if (gamepad1.a) { robot.zeroHeading(); } // Implement zeroHeading in Robot Hardware to call imu.resetYaw()

            // Drive field-centric (rotates axial/lateral by -IMU heading so "forward" = field-forward)
            robot.driveFieldCentric(axial, lateral, yaw);

            // Telemetry for drivers + debugging
            telemetry.addData("Controls", "Drive/Strafe: Left Stick | Turn: Right Stick");
            telemetry.addData("Inputs", "axial=%.2f   lateral=%.2f   yaw=%.2f", axial, lateral, yaw);
            // Optional: expose heading during tuning
            // telemetry.addData("Heading(rad)", robot.getHeadingRadians()); / add a getter in RobotHardware if desired
            telemetry.update();

            // Pace loop-helps with readability and prevents spamming the DS
            sleep(50); // ~20 Hz; TODO(STUDENTS): adjust for your robot feel (e.g., 20-30 ms for snappier control)
        }
    }
}

