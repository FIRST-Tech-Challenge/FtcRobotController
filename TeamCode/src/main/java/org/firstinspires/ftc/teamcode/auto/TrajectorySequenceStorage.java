package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.Wrist;
import org.firstinspires.ftc.teamcode.subsystems.webcam.Webcam;
import org.firstinspires.ftc.teamcode.teamUtil.RobotConfig;
import org.firstinspires.ftc.teamcode.teamUtil.RobotConstants;
import org.firstinspires.ftc.teamcode.teamUtil.odometry.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.teamUtil.odometry.roadrunner.trajectorysequence.TrajectorySequence;

public class TrajectorySequenceStorage {
	
	private TrajectorySequence[] trajectorySequences;
	private int sequenceIndex;
	private RobotConfig r;
	private SampleMecanumDrive drive;
	private Lift lift;
	private Intake intake;
	private Arm arm;
	private Wrist wrist;
	private Webcam webcam;
	
	public static final Pose2d startPoseRight = new Pose2d (34, -65, Math.toRadians(90));

	public static final Pose2d startPoseLeft = new Pose2d (-34, -65, Math.toRadians(90));


	private TrajectorySequence rightStart(){
		drive.setPoseEstimate(startPoseRight);
		return drive.trajectorySequenceBuilder(startPoseRight)
				.UNSTABLE_addTemporalMarkerOffset(0, () -> {
					arm.presetTargetPosition(Arm.ArmPos.FRONT);
					wrist.presetTargetPosition(Wrist.wristPos.FRONT);
					intake.presetTargetPosition(Intake.intakePos.CLOSED);
				})
				.waitSeconds(0.1)
				.forward(5)
				.splineTo(new Vector2d(24, -38),  Math.toRadians(135))
				.splineTo(new Vector2d(8, -30), Math.toRadians(135))
				.UNSTABLE_addTemporalMarkerOffset(-3.5, () -> {
					lift.presetLiftPosition(RobotConstants.poleHeights.HIGH);
				})
				//reach high pole ?
				.setReversed(true)
				.UNSTABLE_addTemporalMarkerOffset(-0.5, () -> {
					lift.presetLiftPosition(RobotConstants.poleHeights.HIGH_DROP);
				})
				.UNSTABLE_addTemporalMarkerOffset(0, () -> {
					intake.presetTargetPosition(Intake.intakePos.OPEN);
					arm.presetTargetPosition(Arm.ArmPos.HALF);
				})
				.UNSTABLE_addTemporalMarkerOffset(1, () -> {
					lift.presetLiftPosition(RobotConstants.poleHeights.GROUND);
				})
				.splineTo(new Vector2d(24, -34),  Math.toRadians(0))
				.splineToSplineHeading(new Pose2d(24, -24, Math.toRadians(0)), Math.toRadians(0))
				.splineToSplineHeading(new Pose2d(54, -12, Math.toRadians(0)), Math.toRadians(0))
				.lineToSplineHeading(new Pose2d(12, -12, Math.toRadians(30)))
				.build();
	}

	private TrajectorySequence rileyRightSection(double xOffset, double yOffset){
		drive.setPoseEstimate(startPoseRight);
		return drive.trajectorySequenceBuilder(startPoseRight)
				.splineTo(new Vector2d(12 + xOffset, -24 + yOffset), Math.toRadians(90))
				.build();
	}
	
	
	
	public TrajectorySequenceStorage leftHigh5(
			RobotConfig r,
			SampleMecanumDrive drive,
			Lift lift,
			Intake intake,
			Arm arm,
			Wrist wrist,
			Webcam webcam
		){
		this.r = r;
		this.drive = drive;
		this.lift = lift;
		this.intake = intake;
		this.arm = arm;
		this.wrist = wrist;
		this.webcam = webcam;
		this.sequenceIndex = 0;
		
		trajectorySequences = new TrajectorySequence[]{
				rileyRightSection(0,0),
				parkingPlaceholder(rileyRightSection(0,0).end())
		};
		
		
		
		return this;
	}

	public TrajectorySequenceStorage rightMedium5(
			RobotConfig r,
			SampleMecanumDrive drive,
			Lift lift,
			Intake intake,
			Arm arm,
			Wrist wrist,
			Webcam webcam
	){
		this.r = r;
		this.drive = drive;
		this.lift = lift;
		this.intake = intake;
		this.arm = arm;
		this.wrist = wrist;
		this.webcam = webcam;
		this.sequenceIndex = 0;

		trajectorySequences = new TrajectorySequence[]{
			rightStart(),
			parkingPlaceholder(rightStart().end())
		};

		return this;
	}
	
	public void followSetSequenceAsync(){
		if(!drive.isBusy()){
			if(sequenceIndex >= trajectorySequences.length - 1) {
				return;
			}
			drive.followTrajectorySequenceAsync(trajectorySequences[++sequenceIndex]);
			
		}
		drive.update();
	}
	
	public void startFollowSetSequenceAsync(){
		if(sequenceIndex >= trajectorySequences.length){
			return;
		}
		drive.followTrajectorySequenceAsync(trajectorySequences[sequenceIndex]);
	}
	
	public void addRightParkSequence(int tagid){
		switch (tagid){
			case 1:
				trajectorySequences[trajectorySequences.length-1] = rightPark1(trajectorySequences[trajectorySequences.length-2].end());
				break;
			case 2:
				trajectorySequences[trajectorySequences.length-1] = rightPark2(trajectorySequences[trajectorySequences.length-2].end());
				break;
			case 3:
				trajectorySequences[trajectorySequences.length-1] = rightPark3(trajectorySequences[trajectorySequences.length-2].end());
				break;
		}
	}
	
	public void addLeftParkSequence(int tagid){
		switch (tagid){
			case 1:
				trajectorySequences[trajectorySequences.length-1] = leftPark1(trajectorySequences[trajectorySequences.length-2].end());
				break;
			case 2:
				trajectorySequences[trajectorySequences.length-1] = leftPark2(trajectorySequences[trajectorySequences.length-2].end());
				break;
			case 3:
				trajectorySequences[trajectorySequences.length-1] = leftPark3(trajectorySequences[trajectorySequences.length-2].end());
				break;
		}
	}
	
	private TrajectorySequence parkingPlaceholder(Pose2d startPose){
		return drive.trajectorySequenceBuilder(startPose)
				.build();
	}
	private TrajectorySequence rightPark1(Pose2d startPose){
		return drive.trajectorySequenceBuilder(startPose)
				.splineTo(new Vector2d(12, -24), Math.toRadians(90)) //TODO fix up values
				.UNSTABLE_addTemporalMarkerOffset(0, () -> {
					arm.presetTargetPosition(Arm.ArmPos.HALF);
					lift.presetLiftPosition(RobotConstants.poleHeights.GROUND);
				})
				.splineTo(new Vector2d(), Math.toRadians(90))
				.build();
	}
	
	private TrajectorySequence rightPark2(Pose2d startPose){
		return drive.trajectorySequenceBuilder(startPose)
				.splineTo(new Vector2d(12, -24), Math.toRadians(90)) //TODO fix up values
				.UNSTABLE_addTemporalMarkerOffset(0, () -> {
					arm.presetTargetPosition(Arm.ArmPos.HALF);
					lift.presetLiftPosition(RobotConstants.poleHeights.GROUND);
				})
				.splineTo(new Vector2d(), Math.toRadians(90))
				.build();
	}
	
	private TrajectorySequence rightPark3(Pose2d startPose){
		return drive.trajectorySequenceBuilder(startPose)
				.splineTo(new Vector2d(12, -24), Math.toRadians(90)) //TODO fix up values
				.UNSTABLE_addTemporalMarkerOffset(0, () -> {
					arm.presetTargetPosition(Arm.ArmPos.HALF);
					lift.presetLiftPosition(RobotConstants.poleHeights.GROUND);
				})
				.splineTo(new Vector2d(), Math.toRadians(90))
				.build();
	}
	
	private TrajectorySequence leftPark1(Pose2d startPose){
		return drive.trajectorySequenceBuilder(startPose)
				.splineTo(new Vector2d(12, -24), Math.toRadians(90)) //TODO fix up values
				.UNSTABLE_addTemporalMarkerOffset(0, () -> {
					arm.presetTargetPosition(Arm.ArmPos.HALF);
					lift.presetLiftPosition(RobotConstants.poleHeights.GROUND);
				})
				.splineTo(new Vector2d(), Math.toRadians(90))
				.build();
	}
	
	private TrajectorySequence leftPark2(Pose2d startPose){
		return drive.trajectorySequenceBuilder(startPose)
				.splineTo(new Vector2d(12, -24), Math.toRadians(90)) //TODO fix up values
				.UNSTABLE_addTemporalMarkerOffset(0, () -> {
					arm.presetTargetPosition(Arm.ArmPos.HALF);
					lift.presetLiftPosition(RobotConstants.poleHeights.GROUND);
				})
				.splineTo(new Vector2d(), Math.toRadians(90))
				.build();
	}
	
	private TrajectorySequence leftPark3(Pose2d startPose){
		return drive.trajectorySequenceBuilder(startPose)
				.splineTo(new Vector2d(12, -24), Math.toRadians(90)) //TODO fix up values
				.UNSTABLE_addTemporalMarkerOffset(0, () -> {
					arm.presetTargetPosition(Arm.ArmPos.HALF);
					lift.presetLiftPosition(RobotConstants.poleHeights.GROUND);
				})
				.splineTo(new Vector2d(), Math.toRadians(90))
				.build();
	}
}
