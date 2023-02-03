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

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name = "Auto", group = "Auto", preselectTeleOp = "Husky TeleOpMode")
public class HuskyLeftAuto extends HuskyAutoBase {
    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();

        waitForStart();
        runtime.reset();

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(-36.00, -64.00, Math.toRadians(90.00));
        drive.setPoseEstimate(startPose);
        TrajectoryVelocityConstraint slowVel = SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL*0.8, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH);

        // region TRAJECTORIES
        // Backup-Plan Trajectory
        Trajectory backupPlanTrajA = drive.trajectoryBuilder(startPose)
                .strafeRight(8)
                .build();
        Trajectory backupPlanTrajB = drive.trajectoryBuilder(backupPlanTrajA.end())
                .strafeLeft(8)
                .build();

        // Place-Preloaded-Cone Trajectory
        TrajectorySequence TRAJ_1 = drive.trajectorySequenceBuilder(new Pose2d(-36.00, -64.00, Math.toRadians(90.00)))
                .setConstraints(slowVel, SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .lineToLinearHeading(new Pose2d(-15.00, -64.00, Math.toRadians(90.00)))
                .lineToLinearHeading(new Pose2d(-10.00, -36.00, Math.toRadians(45.00)))
                .lineToLinearHeading(new Pose2d(-10.01, -36.00, Math.toRadians(90.00)))
                .build();

        // Move-to-Park Trajectory
        TrajectorySequence TRAJ_PARK_1 = drive.trajectorySequenceBuilder(TRAJ_1.end())
                .setConstraints(slowVel, SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .lineToLinearHeading(new Pose2d(-62.00, -36.00, Math.toRadians(90.00)))
                .build();

        TrajectorySequence TRAJ_PARK_2 = drive.trajectorySequenceBuilder(TRAJ_1.end())
                .setConstraints(slowVel, SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .lineToLinearHeading(new Pose2d(-36.00, -36.00, Math.toRadians(90.00)))
                .build();

        TrajectorySequence TRAJ_PARK_3 = drive.trajectorySequenceBuilder(TRAJ_1.end())
                .setConstraints(slowVel, SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .lineToLinearHeading(new Pose2d(-12.00, -36.00, Math.toRadians(90.00)))
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

        switch(this.parkLocation) {
            case LOCATION_1:
                drive.followTrajectorySequence(TRAJ_PARK_1);
                break;
            case LOCATION_2:
                drive.followTrajectorySequence(TRAJ_PARK_2);
                break;
            case LOCATION_3:
                drive.followTrajectorySequence(TRAJ_PARK_3);
                break;
        }

        telemetry.update();
    }
}