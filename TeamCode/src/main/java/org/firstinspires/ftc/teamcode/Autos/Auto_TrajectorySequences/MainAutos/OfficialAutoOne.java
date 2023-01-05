package org.firstinspires.ftc.teamcode.Autos.Auto_TrajectorySequences.MainAutos;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.MechanismTemplates.Arm;
import org.firstinspires.ftc.teamcode.MechanismTemplates.Claw;
import org.firstinspires.ftc.teamcode.MechanismTemplates.Slide;
import org.firstinspires.ftc.teamcode.TeleOps.AprilTags.PowerPlay_AprilTagDetectionDeposit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;


@Autonomous(name = "SexyAuto")
public class OfficialAutoOne extends PowerPlay_AprilTagDetectionDeposit {

	private Arm armControl;
	private Slide slideControl;
	private Claw clawControl;

	public void initialize(){
		armControl = new Arm(hardwareMap);
		slideControl = new Slide(hardwareMap);
		clawControl = new Claw(hardwareMap);
	}

	@Override
	public void runOpMode()
	{
		Pose2d startPose = new Pose2d(61.8,-35,Math.toRadians(180));
		SampleMecanumDrive bot = new SampleMecanumDrive(hardwareMap);
		bot.setPoseEstimate(startPose);

		//Trajectories
		TrajectorySequence terminalAndGoToConeStack = bot.trajectorySequenceBuilder(startPose)
				.strafeTo(new Vector2d(61.8,-10))
				.lineToLinearHeading(new Pose2d(91.8,-12,Math.toRadians(180)))
				.splineToLinearHeading(new Pose2d(110, -22.5, Math.toRadians(90)),Math.toRadians(200))
		        .build();


		waitForStart();
		bot.followTrajectorySequence(terminalAndGoToConeStack);
	}
}
