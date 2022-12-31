package org.firstinspires.ftc.teamcode.Autos.Auto_TrajectorySequences.MainAutos;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.MechanismTemplates.Arm;
import org.firstinspires.ftc.teamcode.MechanismTemplates.Slide;
import org.firstinspires.ftc.teamcode.MechanismTemplates.Claw;
import org.firstinspires.ftc.teamcode.SignalEdgeDetector;
import org.firstinspires.ftc.teamcode.TeleOps.AprilTags.PowerPlay_AprilTagDetection;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous
@Disabled
public class PP_Auto_Quad2 extends PowerPlay_AprilTagDetection
{
	private boolean isAuto = true;
	//Declaring lift motor powers
	private Arm armControl;
	private Slide slideControl;
	private Claw clawMovement;

	SignalEdgeDetector isIntakePosition;

	// Declaring our motor PID for the lift; passing through our PID values
	public PP_Auto_Quad2()
	{
		super.runOpMode(); // runs the opMode of the apriltags pipeline

		waitForStart();

		armControl = new Arm(hardwareMap);
		slideControl = new Slide(hardwareMap);
		clawMovement = new Claw(hardwareMap,() -> gamepad2.right_bumper, () -> gamepad2.a);
	}

	@Override
	public void runOpMode()
	{
		// bot object created
		SampleMecanumDrive bot = new SampleMecanumDrive(hardwareMap);

		// calling our method to run trajectories and passing through our newly created bot object
		bot = runTrajectories(bot);

		// LOOPS TO RUN ASYNC \\
		while (opModeIsActive()){
			bot.update();
			slideControl.update(telemetry);
			armControl.update(telemetry);
		}

	}// end of runOpMode()

	public SampleMecanumDrive runTrajectories(SampleMecanumDrive bot){
		Pose2d startPose = new Pose2d(35, 61.8, Math.toRadians(90));
		bot.setPoseEstimate(startPose);

		// TRAJECTORY SEQUENCES \\
		Pose2d endPose = cycle(bot, startPose);


		if(aprilTag_ID == 1) {
			TrajectorySequence tag1 = bot.trajectorySequenceBuilder(endPose)
					.lineTo(new Vector2d(59, 35))
					.build();
			bot.followTrajectorySequenceAsync(tag1);
		}
		else if(aprilTag_ID == 2) {
			telemetry.addData("Playboy Parti","Currently Performing at the local retirement center");

		}
		else{
			TrajectorySequence tag3 = bot.trajectorySequenceBuilder(endPose)
					.lineTo(new Vector2d(11.8,8))
					.build();
			bot.followTrajectorySequenceAsync(tag3);
		}
		return bot;
	}// end of runTrajectories

	public Pose2d cycle(SampleMecanumDrive bot, Pose2d currentPosition){
		TrajectorySequence openingMove =  bot.trajectorySequenceBuilder(currentPosition)
				.addTemporalMarker(2,() -> {
					slideControl.setHighJunction();
					//slideControl.Update();
					armControl.setIntake();
					//armControl.update();
					clawMovement.toggleOpenClose();
					armControl.setExtake();

				})//temporal marker for the extake
				.lineToLinearHeading(new Pose2d(32,8,Math.toRadians(50)))

				.addTemporalMarker(1,()->{
					slideControl.setIntakeOrGround();
					//slideControl.Update();
					clawMovement.toggleOpenClose();
					armControl.setIntake();
					//armControl.update();
				})//marker for the intake, the timing will be tested so where the markers are located and times are subject to change

				.lineToLinearHeading(new Pose2d(57, 12.3, Math.toRadians(0)))

				//.addTemporalMarker()
				.lineToLinearHeading(new Pose2d(32, 8, Math.toRadians(50)))
				.waitSeconds(1)//Deposit at junction; Under the impression that using the async PID, the slides will be already be moved up
				.build();

		Pose2d endPose = new Pose2d(32,8,Math.toRadians(50));

		TrajectorySequence cycles =  bot.trajectorySequenceBuilder(currentPosition)
				.addTemporalMarker(3,() -> {
					slideControl.setIntakeOrGround();
					//slideControl.Update();
					clawMovement.toggleOpenClose();
				})
				.lineToLinearHeading(new Pose2d(57, 12.3, Math.toRadians(0)))
				.lineToLinearHeading(new Pose2d(32, 8, Math.toRadians(50)))
				.waitSeconds(1)//Under the impression that using the async PID, the slides will be already be moved up
				.build();

		bot.followTrajectorySequenceAsync(openingMove);
		for(int i = 1; i <= 3; i++){
			bot.followTrajectorySequenceAsync(cycles);
		}
		return endPose;
	}
}

