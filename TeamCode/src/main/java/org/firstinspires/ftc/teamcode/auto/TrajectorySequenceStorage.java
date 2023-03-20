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
	
	public static final Pose2d startPoseLeft = new Pose2d (34, -65, Math.toRadians(90));
	
	private TrajectorySequence leftStart(){
		drive.setPoseEstimate(startPoseLeft);
		return drive.trajectorySequenceBuilder(startPoseLeft)
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
				leftStart()
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
	
	public void printSequence(){
		r.opMode.telemetry.addData("length", trajectorySequences.length);
		for (int i = 0, trajectorySequencesLength = trajectorySequences.length; i < trajectorySequencesLength; i++) {
			TrajectorySequence trajectorySequenced = trajectorySequences[i];
			r.opMode.telemetry.addData(String.valueOf(i), trajectorySequenced);
		}
	}
}
