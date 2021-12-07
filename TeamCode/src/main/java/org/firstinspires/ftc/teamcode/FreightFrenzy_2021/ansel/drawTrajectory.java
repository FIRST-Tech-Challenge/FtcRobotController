package org.firstinspires.ftc.teamcode.FreightFrenzy_2021.ansel;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
import static org.firstinspires.ftc.teamcode.FreightFrenzy_2021.ansel.PoseStorage.state;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;

public class drawTrajectory {
    public static String findTeamState() {
        if (state == driveMethod.poseState.BLUE || state == driveMethod.poseState.BLUE_WAREHOUSE || state == driveMethod.poseState.BLUE_OTHERS || state == driveMethod.poseState.BLUE_STORAGEUNIT || state == driveMethod.poseState.BLUE_SHARED_HUB) {
            return "BLUE";
        } else if (state == driveMethod.poseState.RED || state == driveMethod.poseState.RED_WAREHOUSE || state == driveMethod.poseState.RED_OTHERS || state == driveMethod.poseState.RED_STORAGEUNIT || state == driveMethod.poseState.RED_SHARED_HUB) {
            return "RED";
        } else {
            return "UNKNOWN";
        }
    }

    public static void gotoSharedHub() {
        SampleMecanumDrive chassis = new SampleMecanumDrive(hardwareMap);
        chassis.setPoseEstimate(PoseStorage.currentPose);

        String poseState = findTeamState();

        switch (poseState) {
            case "BLUE": {
                chassis.setPoseEstimate(fieldConstant.SHARED_BLUE_ENTER_POSE);
                Trajectory sharedTraj1 = chassis.trajectoryBuilder(chassis.getPoseEstimate(), true)
                        .lineTo(new Vector2d(64.75, 18))
                        .build();
                chassis.followTrajectory(sharedTraj1);
                Trajectory sharedTraj2 = chassis.trajectoryBuilder(sharedTraj1.end(), true)
                        .lineToLinearHeading(fieldConstant.SHARED_BLUE_END_POSE)
                        .build();
                chassis.followTrajectory(sharedTraj2);

                telemetry.addLine("Shared Hub - Blue");
                break;
            }
            case "RED": {
                chassis.setPoseEstimate(fieldConstant.SHARED_RED_ENTER_POSE);
                Trajectory sharedTraj1 = chassis.trajectoryBuilder(chassis.getPoseEstimate(), true)
                        .lineTo(new Vector2d(64.75, -18))
                        .build();
                chassis.followTrajectory(sharedTraj1);
                Trajectory sharedTraj2 = chassis.trajectoryBuilder(sharedTraj1.end(), true)
                        .lineToLinearHeading(fieldConstant.SHARED_RED_END_POSE)
                        .build();
                chassis.followTrajectory(sharedTraj2);

                
                telemetry.addLine("Shared Hub - Red");
                break;
            }
            case "UNKNOWN":
                telemetry.addLine("Unknown position, cannot travel to Shared Hub.");
                break;
            default:
                telemetry.addLine("Position Error");
                break;
        }
    }

    public static void gotoPlate() {
        SampleMecanumDrive chassis = new SampleMecanumDrive(hardwareMap);
        chassis.setPoseEstimate(PoseStorage.currentPose);

        String poseState = findTeamState();

        switch (poseState) {
            case "BLUE": {
                Pose2d poseEstimate = chassis.getPoseEstimate();
                Pose2d targetBlue = driveMethod.targetPlatePose(poseEstimate, fieldConstant.BLUE_PLATE, 9 + 8.75 + 2);

                Trajectory bluehubTraj = chassis.trajectoryBuilder(poseEstimate,true)
                        .splineTo(new Vector2d(targetBlue.getX(), targetBlue.getY()), Math.PI+ driveMethod.targetAngle(true, poseEstimate, fieldConstant.BLUE_PLATE))
                        .build();
                chassis.followTrajectory(bluehubTraj);

                telemetry.addLine("Alliance Hub - Blue");
                break;
            }
            case "RED": {
                Pose2d poseEstimate = chassis.getPoseEstimate();
                Pose2d targetRed = driveMethod.targetPlatePose(poseEstimate, fieldConstant.RED_PLATE, 9 + 8.75 + 2);

                Trajectory bluehubTraj = chassis.trajectoryBuilder(poseEstimate,true)
                        .splineTo(new Vector2d(targetRed.getX(), targetRed.getY()), Math.PI+ driveMethod.targetAngle(true, poseEstimate, fieldConstant.RED_PLATE))
                        .build();
                chassis.followTrajectory(bluehubTraj);

                telemetry.addLine("Alliance Hub - Red");
                break;
            }
            case "UNKNOWN":
                telemetry.addLine("Unknown position, cannot travel to alliance hub.");
                break;
            default:
                telemetry.addLine("Position Error");
                break;
        }
    }

    public static void updatePose(){
        SampleMecanumDrive chassis = new SampleMecanumDrive(hardwareMap);
        chassis.setPoseEstimate(PoseStorage.currentPose);
        chassis.updatePoseEstimate();
        PoseStorage.state = driveMethod.fieldState(chassis.getPoseEstimate());
    }

}
