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


@Autonomous(name = "RedTerminalCycleAuto")
@Config
public class OfficialAutoOne extends PowerPlay_AprilTagDetectionDeposit {

	private Arm armControl;
	private Slide slideControl;
	private Claw clawControl;

	public static double x = 46;
	public static double y = 24;
	public static double angle = 91.45;
	public static double strafe = 14;


	public void initialize(){
		armControl = new Arm(hardwareMap);
		slideControl = new Slide(hardwareMap);
		clawControl = new Claw(hardwareMap);
	}

	@Override
	public void runOpMode()
	{
		Pose2d startPose = new Pose2d(0,0,Math.toRadians(180));
		SampleMecanumDrive bot = new SampleMecanumDrive(hardwareMap);
		bot.setPoseEstimate(startPose);

		//Trajectories
		initialize();


		TrajectorySequence junction = bot.trajectorySequenceBuilder(startPose)
				.lineToLinearHeading(new Pose2d(60, -5.25, Math.toRadians(90))) //If 13.53 V, y = -3.5
				.UNSTABLE_addTemporalMarkerOffset(0,()->{
					slideControl.setHighJunction(telemetry);
					armControl.setExtake();
					clawControl.toggleWristRotate();
					//slideControl.setIntakeOrGround();
					//armControl.setIntake();
				})
				.waitSeconds(0.75)
				.UNSTABLE_addTemporalMarkerOffset(0,()->{
					slideControl.setCustom(1850);
				})
				.waitSeconds(.5)
				.UNSTABLE_addTemporalMarkerOffset(0,()->{
					clawControl.toggleAutoOpenClose();

				})
				.waitSeconds(0.15)
				//.splineToConstantHeading(new Vector2d(x,y),Math.toRadians(angle))
				.build();

		TrajectorySequence goToConeStack = bot.trajectorySequenceBuilder(junction.end())
				.strafeLeft(13)
				.lineToLinearHeading(new Pose2d(x, y, Math.toRadians(angle)))
				.waitSeconds(1)
				.build();

		TrajectorySequence conesToJunction = bot.trajectorySequenceBuilder(goToConeStack.end())
				.lineToLinearHeading(new Pose2d(47,-4.5,Math.toRadians(90)))
				.strafeRight(13)
				//.lineToLinearHeading(new Pose2d(60,-4.5,Math.toRadians(90)))
				.build();

		waitForStart();


		bot.followTrajectorySequenceAsync(junction);
		//bot.followTrajectorySequence(goToConeStack);
		//bot.followTrajectorySequence(conesToJunction);

		while(opModeIsActive()  && !isStopRequested()){
			bot.update();
			armControl.update(telemetry);
			telemetry.addLine("run");
			telemetry.update();
			slideControl.update(telemetry);
		}

	}
}
