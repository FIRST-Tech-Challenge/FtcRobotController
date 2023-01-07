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

	public static double x = 32;
	public static double y = 24;
	public static double angle = 91.45;
	public static double strafe = 14;


	public void initialize(){
		armControl = new Arm(hardwareMap);
		slideControl = new Slide(hardwareMap);
		//clawControl = new Claw(hardwareMap);
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
				.lineToLinearHeading(new Pose2d(45, -4.5, Math.toRadians(90))) //If 13.53 V, y = -3.5
				.waitSeconds(2)
				.strafeLeft(13)
				.lineToLinearHeading(new Pose2d(x, y, Math.toRadians(angle)))
				.waitSeconds(1)
				//.splineToConstantHeading(new Vector2d(x,y),Math.toRadians(angle))
				.build();



		while(!opModeIsActive()){
			telemetry.addData("In init loop", "");
			telemetry.update();
		}
		bot.followTrajectorySequence(terminalAndGoToConeStack);
	}
}
