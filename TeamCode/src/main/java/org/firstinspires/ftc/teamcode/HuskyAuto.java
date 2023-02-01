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

import static org.firstinspires.ftc.teamcode.HuskyBot.CLAW_GRAB_CLOSE_POSITION;
import static org.firstinspires.ftc.teamcode.HuskyBot.CLAW_LIFT_START_POSITION;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name = "Auto", group = "Auto", preselectTeleOp = "Husky TeleOpMode")
public class HuskyAuto extends HuskyAutoBase {
    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();

        waitForStart();
        runtime.reset();

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        // region TRAJECTORIES
        // Backup-Plan Trajectory
        Trajectory backupPlanTrajA = drive.trajectoryBuilder(new Pose2d(-36, -61.00, Math.toRadians(90.00)))
                .strafeRight(8)
                .build();
        Trajectory backupPlanTrajB = drive.trajectoryBuilder(backupPlanTrajA.end())
                .strafeLeft(8)
                .build();

        // Place-Preloaded-Cone Trajectory
        TrajectorySequence TRAJ_1 = drive.trajectorySequenceBuilder(new Pose2d(-36.00, -61.00, Math.toRadians(90.00)))
                .splineTo(new Vector2d(-18.51, -61.03), Math.toRadians(0.00))
                .splineTo(new Vector2d(-8.91, -37.94), Math.toRadians(65.14))
                .addDisplacementMarker(() -> {})
                .waitSeconds(1)
                .build();

        // Move-to-Stack Trajectory
        TrajectorySequence TRAJ_2 = drive.trajectorySequenceBuilder(new Pose2d(-8.91, -37.94, Math.toRadians(39.81)))
                .splineTo(new Vector2d(-18.97, -11.43), Math.toRadians(172.23))
                .splineTo(new Vector2d(-57.60, -11.20), Math.toRadians(180.00))
                .addDisplacementMarker(() -> {})
                .build();
        // endregion

        huskyBot.clawLift.setPosition(1.0);
        huskyBot.clawGrab.setPosition(CLAW_GRAB_CLOSE_POSITION);

        this.parkLocation = getParkLocation();



        // Backup plan for apriltag detection (move the robot little bit right and try to detect again
        if(this.parkLocation == Location.LOCATION_0){
            drive.followTrajectory(backupPlanTrajA);
            this.parkLocation = getParkLocation();
            drive.followTrajectory(backupPlanTrajB);
        }

        // Set the park location to 2 if it couldn't detect the apriltag even after the backup plan
        if(this.parkLocation == Location.LOCATION_0){
            telemetry.addLine("No detection!");
            this.parkLocation = Location.LOCATION_2;
        }

        drive.followTrajectorySequence(TRAJ_1);
        drive.followTrajectorySequence(TRAJ_2);

        telemetry.update();
    }
}