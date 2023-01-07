package org.firstinspires.ftc.teamcode.Autos.Auto_TrajectorySequences.MainAutos;

import com.acmerobotics.dashboard.config.Config;
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
@Config
public class OfficialAutoOne extends PowerPlay_AprilTagDetectionDeposit {

	private Arm armControl;
	private Slide slideControl;
	private Claw clawControl;

	public static double x = 56;
	public static double y = -6;
	public static double angle = 25;
	public static double angle2 = 87;

	public void initialize(){
		/*
		armControl = new Arm(hardwareMap);
		slideControl = new Slide(hardwareMap);
		clawControl = new Claw(hardwareMap);
		 */
	}

	@Override
	public void runOpMode()
	{
		Pose2d startPose = new Pose2d(0,0,Math.toRadians(180));
		SampleMecanumDrive bot = new SampleMecanumDrive(hardwareMap);
		bot.setPoseEstimate(startPose);

		//Trajectories
		initialize();


		TrajectorySequence terminalAndGoToConeStack = bot.trajectorySequenceBuilder(startPose)
				.lineToLinearHeading(new Pose2d(60, -5, Math.toRadians(90)))
				.lineToLinearHeading(new Pose2d(45.5, -5, Math.toRadians(90)))
				.lineToLinearHeading(new Pose2d(47.75, 22, Math.toRadians(87)))
				.splineToConstantHeading(new Vector2d(56, -6), Math.toRadians(25))
				.build();

		/*TrajectorySequence splineTest = bot.trajectorySequenceBuilder(terminalAndGoToConeStack.end())
				.splineToLinearHeading(new Pose2d(x, y,Math.toRadians(angle2)), Math.toRadians(angle))
				.build();
		*/

		while(!opModeIsActive()){
			telemetry.addData("In init loop", "");
			telemetry.update();
		}
		bot.followTrajectorySequence(terminalAndGoToConeStack);
	}
}
