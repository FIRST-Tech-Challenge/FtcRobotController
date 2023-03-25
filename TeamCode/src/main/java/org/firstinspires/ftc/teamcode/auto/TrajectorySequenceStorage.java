package org.firstinspires.ftc.teamcode.auto;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TranslationalVelocityConstraint;

import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.Wrist;
import org.firstinspires.ftc.teamcode.subsystems.webcam.Webcam;
import org.firstinspires.ftc.teamcode.teamUtil.RobotConfig;
import org.firstinspires.ftc.teamcode.teamUtil.odometry.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.teamUtil.odometry.roadrunner.trajectorysequence.TrajectorySequence;

import java.util.ArrayList;
import java.util.Arrays;

public class TrajectorySequenceStorage {
	
	private TrajectorySequence[] trajectorySequences;
	private ArrayList<TrajectorySequence> trajectorySequenceArrayList;
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


	private TrajectorySequence rightStartCloseHighPole(){
		drive.setPoseEstimate(startPoseRight);
		return drive.trajectorySequenceBuilder(startPoseRight)
				.UNSTABLE_addTemporalMarkerOffset(0, () -> {
					arm.presetTargetPosition(Arm.ArmPos.FRONT_DELIVERY);
					r.setPickup(true);
					wrist.presetTargetPosition(Wrist.WristPos.FRONT);
					intake.presetTargetPosition(Intake.IntakePos.CLOSED);
				})
				.waitSeconds(0.1)
				.UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
					lift.presetLiftPosition(Lift.PoleHeights.STACK4);
				})
				.splineTo(new Vector2d(36, -12),  Math.toRadians(90))//drive forward
				.turn(Math.toRadians(-90))//turn
				.setReversed(true)
				.splineTo(new Vector2d(10, -12), Math.toRadians(180))//drive to pole
				.UNSTABLE_addTemporalMarkerOffset(-0.8, () -> {
					lift.presetLiftPosition(Lift.PoleHeights.HIGH);
					wrist.presetTargetPosition(Wrist.WristPos.BACK);
					arm.presetTargetPosition(Arm.ArmPos.BACK_DELIVERY);
				})
				.setVelConstraint(new MinVelocityConstraint(Arrays.asList(
						new TranslationalVelocityConstraint(20),
						new AngularVelocityConstraint(1)
				)))
				.splineTo(new Vector2d(6, -13), Math.toRadians(210))//drive to pole
				.waitSeconds(0)
				.UNSTABLE_addTemporalMarkerOffset(0, () -> {
					lift.presetLiftPosition(Lift.PoleHeights.HIGH_DROP);
				})
				.resetConstraints()
		        .build();
	}

	private TrajectorySequence rightCloseHighPoleToStack(Lift.PoleHeights poleHeight){
		return drive.trajectorySequenceBuilder(trajectorySequenceArrayList.get(trajectorySequenceArrayList.size()-1).end())
				.setReversed(false)
				.waitSeconds(0.1)
				.UNSTABLE_addTemporalMarkerOffset(0, () -> {
					intake.presetTargetPosition(Intake.IntakePos.OPEN);
				})
				.UNSTABLE_addTemporalMarkerOffset(0.4, () -> {
					lift.presetLiftPosition(poleHeight);
					intake.presetTargetPosition(Intake.IntakePos.CLOSED);
					wrist.presetTargetPosition(Wrist.WristPos.FRONT);
					arm.presetTargetPosition(Arm.ArmPos.HALF);
				})
				.splineTo(new Vector2d(36, -12.5), Math.toRadians(0))//drive back to centre
				.splineTo(new Vector2d(54.4, -8.2), Math.toRadians(10))//drive to stack
				.waitSeconds(0.0)
				.UNSTABLE_addTemporalMarkerOffset(-0.8, () -> {
					intake.presetTargetPosition(Intake.IntakePos.OPEN);
					wrist.presetTargetPosition(Wrist.WristPos.FRONT);
					arm.presetTargetPosition(Arm.ArmPos.FRONT_DELIVERY);
				})
				.build();
	}
	
	private TrajectorySequence rightStackToCloseHighPole(){
		return drive.trajectorySequenceBuilder(trajectorySequenceArrayList.get(trajectorySequenceArrayList.size()-1).end())
				.setReversed(true)
				.UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
					intake.presetTargetPosition(Intake.IntakePos.CLOSED);
				})
				.UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
					arm.presetTargetPosition(Arm.ArmPos.HALF);
					wrist.presetTargetPosition(Wrist.WristPos.BACK);
				})
				.waitSeconds(0.3)
				.splineTo(new Vector2d(36, -12), Math.toRadians(180))//drive back to centre
				.splineTo(new Vector2d(10, -12), Math.toRadians(180))//drive to pole
				.UNSTABLE_addTemporalMarkerOffset(-0.8, () -> {
					lift.presetLiftPosition(Lift.PoleHeights.HIGH);
					wrist.presetTargetPosition(Wrist.WristPos.BACK);
					arm.presetTargetPosition(Arm.ArmPos.BACK_DELIVERY);
				})
				.setVelConstraint(new MinVelocityConstraint(Arrays.asList(
						new TranslationalVelocityConstraint(20),
						new AngularVelocityConstraint(1)
				)))
				.splineTo(new Vector2d(6, -13), Math.toRadians(210))//drive to pole
				.waitSeconds(0)
				
				.UNSTABLE_addTemporalMarkerOffset(0, () -> {
					lift.presetLiftPosition(Lift.PoleHeights.HIGH_DROP);
				})
				.resetConstraints()
				.build();
	}
	
	private TrajectorySequence rightStartMediumPole(){
		drive.setPoseEstimate(startPoseRight);
		return drive.trajectorySequenceBuilder(startPoseRight)
				.UNSTABLE_addTemporalMarkerOffset(0, () -> {
					arm.presetTargetPosition(Arm.ArmPos.BACK_DELIVERY);
					wrist.presetTargetPosition(Wrist.WristPos.BACK);
					intake.presetTargetPosition(Intake.IntakePos.CLOSED);
				})
				.waitSeconds(0.2)
				.splineTo(new Vector2d(36, -12),  Math.toRadians(90))//drive forward
				.UNSTABLE_addTemporalMarkerOffset(-0.6, () -> {
					lift.presetLiftPosition(Lift.PoleHeights.MEDIUM);
				})
				.turn(Math.toRadians(-70))//turn
				.build();
	}
	
	private TrajectorySequence rightMediumPoleToStack(Lift.PoleHeights poleHeight){
		return drive.trajectorySequenceBuilder(trajectorySequenceArrayList.get(trajectorySequenceArrayList.size()-1).end())
				.setReversed(false)
				.waitSeconds(0.1)
				.UNSTABLE_addTemporalMarkerOffset(-0.1, () -> {
					lift.presetLiftPosition(Lift.PoleHeights.HIGH_DROP);
					intake.presetTargetPosition(Intake.IntakePos.OPEN);
				})
				.splineTo(new Vector2d(55, -7), 0)//drive to stack
				.UNSTABLE_addTemporalMarkerOffset(-0.3, () -> {
					lift.presetLiftPosition(poleHeight);
					intake.presetTargetPosition(Intake.IntakePos.CLOSED);
					arm.presetTargetPosition(Arm.ArmPos.HALF);
				})
				.waitSeconds(0)
				.UNSTABLE_addTemporalMarkerOffset(-0.5, () -> {
					intake.presetTargetPosition(Intake.IntakePos.OPEN);
					wrist.presetTargetPosition(Wrist.WristPos.BACK);
					arm.presetTargetPosition(Arm.ArmPos.BACK_DELIVERY);
				})
				.build();
	}
	
	private TrajectorySequence rightStackToMediumPole(){
		return drive.trajectorySequenceBuilder(trajectorySequenceArrayList.get(trajectorySequenceArrayList.size()-1).end())
				.setReversed(true)
				.UNSTABLE_addTemporalMarkerOffset(0, () -> {
					intake.presetTargetPosition(Intake.IntakePos.CLOSED);
				})
				.UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
					arm.presetTargetPosition(Arm.ArmPos.HALF);
					wrist.presetTargetPosition(Wrist.WristPos.FRONT);
				})
				.splineTo(new Vector2d(36, -12), Math.toRadians(210))//drive back to centre
				.waitSeconds(0)
				.UNSTABLE_addTemporalMarkerOffset(-0.5, () -> {
					lift.presetLiftPosition(Lift.PoleHeights.HIGH);
					wrist.presetTargetPosition(Wrist.WristPos.FRONT);
					arm.presetTargetPosition(Arm.ArmPos.FRONT_DELIVERY);
				})
				.build();
	}
	
	public TrajectorySequenceStorage rightCloseHigh5(
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
		
		trajectorySequenceArrayList = new ArrayList<>();
		trajectorySequenceArrayList.add(rightStartCloseHighPole());
		
		trajectorySequenceArrayList.add(rightCloseHighPoleToStack(Lift.PoleHeights.STACK4));
		trajectorySequenceArrayList.add(rightStackToCloseHighPole());
		
		trajectorySequenceArrayList.add(rightCloseHighPoleToStack(Lift.PoleHeights.STACK3));
		trajectorySequenceArrayList.add(rightStackToCloseHighPole());
		
		trajectorySequenceArrayList.add(rightCloseHighPoleToStack(Lift.PoleHeights.STACK2));
		trajectorySequenceArrayList.add(rightStackToCloseHighPole());
		
		trajectorySequenceArrayList.add(rightCloseHighPoleToStack(Lift.PoleHeights.STACK1));
		trajectorySequenceArrayList.add(rightStackToCloseHighPole());
		
		trajectorySequenceArrayList.add(parkingPlaceholder());
		
		trajectorySequences = new TrajectorySequence[trajectorySequenceArrayList.size()];
		for (int i = 0; i < trajectorySequenceArrayList.size(); i++) {
			trajectorySequences[i] = trajectorySequenceArrayList.get(i);
		}
		
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
		
		trajectorySequenceArrayList = new ArrayList<>();
		trajectorySequenceArrayList.add(rightStartMediumPole());
		
		trajectorySequenceArrayList.add(rightMediumPoleToStack(Lift.PoleHeights.STACK4));
		trajectorySequenceArrayList.add(rightStackToMediumPole());
		
		trajectorySequenceArrayList.add(rightMediumPoleToStack(Lift.PoleHeights.STACK3));
		trajectorySequenceArrayList.add(rightStackToMediumPole());
		
		trajectorySequenceArrayList.add(rightMediumPoleToStack(Lift.PoleHeights.STACK2));
		trajectorySequenceArrayList.add(rightStackToMediumPole());
		
		trajectorySequenceArrayList.add(rightMediumPoleToStack(Lift.PoleHeights.STACK1));
		trajectorySequenceArrayList.add(rightStackToMediumPole());
		
		trajectorySequenceArrayList.add(rightMediumPoleToStack(Lift.PoleHeights.STACK0));
		trajectorySequenceArrayList.add(rightStackToMediumPole());
		
		trajectorySequenceArrayList.add(parkingPlaceholder());
		
		trajectorySequences = new TrajectorySequence[trajectorySequenceArrayList.size()];
		for (int i = 0; i < trajectorySequenceArrayList.size(); i++) {
			trajectorySequences[i] = trajectorySequenceArrayList.get(i);
		}

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
			default:
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
			default:
				trajectorySequences[trajectorySequences.length-1] = leftPark3(trajectorySequences[trajectorySequences.length-2].end());
				break;
		}
	}
	
	private TrajectorySequence parkingPlaceholder(){
		return drive.trajectorySequenceBuilder(trajectorySequenceArrayList.get(trajectorySequenceArrayList.size()-1).end())
				.UNSTABLE_addTemporalMarkerOffset(0, () -> {
					intake.presetTargetPosition(Intake.IntakePos.OPEN);
				})
				.back(1)
				.UNSTABLE_addTemporalMarkerOffset(0, () -> {
					lift.presetLiftPosition(Lift.PoleHeights.GROUND);
					intake.presetTargetPosition(Intake.IntakePos.CLOSED);
					arm.presetTargetPosition(Arm.ArmPos.HALF);
				})
				.build();
	}
	private TrajectorySequence rightPark1(Pose2d startPose){
		return drive.trajectorySequenceBuilder(startPose)
				.UNSTABLE_addTemporalMarkerOffset(0, () -> {
					intake.presetTargetPosition(Intake.IntakePos.OPEN);
				})
				.waitSeconds(0.1)
				.UNSTABLE_addTemporalMarkerOffset(0.2, () -> {
					arm.presetTargetPosition(Arm.ArmPos.HALF);
					lift.presetLiftPosition(Lift.PoleHeights.GROUND);
				})
				.splineTo(new Vector2d(12, -12), Math.toRadians(90)) //TODO fix up values
				.setReversed(true)
				.splineTo(new Vector2d(12, -24), Math.toRadians(270))
				.build();
	}
	
	private TrajectorySequence rightPark2(Pose2d startPose){
		return drive.trajectorySequenceBuilder(startPose)
				.UNSTABLE_addTemporalMarkerOffset(0, () -> {
					intake.presetTargetPosition(Intake.IntakePos.OPEN);
				})
				.waitSeconds(0.1)
				.UNSTABLE_addTemporalMarkerOffset(0.2, () -> {
					arm.presetTargetPosition(Arm.ArmPos.HALF);
					lift.presetLiftPosition(Lift.PoleHeights.GROUND);
				})
				.splineTo(new Vector2d(36, -12), Math.toRadians(90)) //TODO fix up values
				.setReversed(true)
				.splineTo(new Vector2d(36, -24), Math.toRadians(270))
				.build();
	}
	
	private TrajectorySequence rightPark3(Pose2d startPose){
		return drive.trajectorySequenceBuilder(startPose)
				.UNSTABLE_addTemporalMarkerOffset(0, () -> {
					intake.presetTargetPosition(Intake.IntakePos.OPEN);
				})
				.waitSeconds(0.1)
				.UNSTABLE_addTemporalMarkerOffset(0.2, () -> {
					arm.presetTargetPosition(Arm.ArmPos.HALF);
					lift.presetLiftPosition(Lift.PoleHeights.GROUND);
				})
				.splineTo(new Vector2d(60, -12), Math.toRadians(90)) //TODO fix up values
				.setReversed(true)
				.splineTo(new Vector2d(60, -24), Math.toRadians(270))
				.build();
	}
	
	private TrajectorySequence leftPark1(Pose2d startPose){
		return drive.trajectorySequenceBuilder(startPose)
				.UNSTABLE_addTemporalMarkerOffset(0, () -> {
					intake.presetTargetPosition(Intake.IntakePos.OPEN);
				})
				.waitSeconds(0.1)
				.UNSTABLE_addTemporalMarkerOffset(0.2, () -> {
					arm.presetTargetPosition(Arm.ArmPos.HALF);
					lift.presetLiftPosition(Lift.PoleHeights.GROUND);
				})
				.splineTo(new Vector2d(-12, -12), Math.toRadians(90)) //TODO fix up values
				.setReversed(true)
				.splineTo(new Vector2d(-12, -24), Math.toRadians(270))
				.build();
	}
	
	private TrajectorySequence leftPark2(Pose2d startPose){
		return drive.trajectorySequenceBuilder(startPose)
				.UNSTABLE_addTemporalMarkerOffset(0, () -> {
					intake.presetTargetPosition(Intake.IntakePos.OPEN);
				})
				.waitSeconds(0.1)
				.UNSTABLE_addTemporalMarkerOffset(0.2, () -> {
					arm.presetTargetPosition(Arm.ArmPos.HALF);
					lift.presetLiftPosition(Lift.PoleHeights.GROUND);
				})
				.splineTo(new Vector2d(-36, -12), Math.toRadians(90)) //TODO fix up values
				.setReversed(true)
				.splineTo(new Vector2d(-36, -24), Math.toRadians(270))
				.build();
	}
	
	private TrajectorySequence leftPark3(Pose2d startPose){
		return drive.trajectorySequenceBuilder(startPose)
				.UNSTABLE_addTemporalMarkerOffset(0, () -> {
					intake.presetTargetPosition(Intake.IntakePos.OPEN);
				})
				.waitSeconds(0.1)
				.UNSTABLE_addTemporalMarkerOffset(0.2, () -> {
					arm.presetTargetPosition(Arm.ArmPos.HALF);
					lift.presetLiftPosition(Lift.PoleHeights.GROUND);
				})
				.splineTo(new Vector2d(-60, -12), Math.toRadians(90)) //TODO fix up values
				.setReversed(true)
				.splineTo(new Vector2d(-60, -24), Math.toRadians(270))
				.build();
	}
}
