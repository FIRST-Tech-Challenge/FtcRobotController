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
package org.firstinspires.ftc.teamcode.Autonomous

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.DriveMethods
import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive
import java.util.Locale

/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive TeleOp for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */
// Autonomous
@TeleOp(name = "AutoRedBack", group = "Linear Opmode")
class AutoRedBack : DriveMethods() {
    override fun runOpMode() {
        // Setup Odometry :)
        val drive = SampleMecanumDrive(hardwareMap)

        // Setup up the trajectory sequence (drive path)
        val traj1 = drive.trajectorySequenceBuilder(Pose2d())
            .strafeRight(10.0)
            .forward(5.0)
            .splineTo(Vector2d(12.0, 20.0), Math.toRadians(90.0))
            .forward(5.0)
            .splineTo(Vector2d(0.0, 38.0), Math.toRadians(180.0))
            .splineTo(Vector2d(-12.0, 20.0), Math.toRadians(-90.0))
            .forward(10.0)
            .splineTo(Vector2d(0.0, -10.0), Math.toRadians(0.0))
            .strafeLeft(10.0)
            .build()

        // Tell the User the Robot has been initialized
        telemetry.addData("Status", "Initialized")
        telemetry.update()


        // Wait for the game to start (driver presses PLAY)
        waitForStart()
        telemetry.addLine("started")
        telemetry.update()
        if (isStopRequested) return
        telemetry.addLine("not stopped")
        telemetry.update()
        drive.followTrajectorySequenceAsync(traj1)
        while (opModeIsActive() && !isStopRequested) {
            drive.update()
        }
        telemetry.addLine("tried to run code")
        telemetry.update()
        sleep(1000)
        val (x, y, heading) = drive.poseEstimate
        telemetry.addData(
            "Current Position",
            String.format(Locale.ENGLISH, "X: %f, Y: %f, and Rotation: %f", x, y, heading)
        )
        telemetry.update()
    }
}