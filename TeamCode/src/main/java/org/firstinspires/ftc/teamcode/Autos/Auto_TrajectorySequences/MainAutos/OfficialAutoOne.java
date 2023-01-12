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

	public static int x = 64;
	public static double y = -3.75;
	public static int firstLinetoHeadingX = 60;
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
				//If 13.53 V, y = -3.5

				//High Junction with Preload
				.UNSTABLE_addTemporalMarkerOffset(0.75,()->{
					slideControl.setHighJunction(telemetry);
					armControl.setExtake();
					clawControl.toggleWristRotate();
					//slideControl.setIntakeOrGround();
					//armControl.setIntake();
				})
				.lineToLinearHeading(new Pose2d(x, y, Math.toRadians(90))) //preload
				//moves down after flipping
				.UNSTABLE_addTemporalMarkerOffset(0,()->{
					slideControl.setCustom(1450);
				})
				.waitSeconds(.5)
				//opens claw
				.UNSTABLE_addTemporalMarkerOffset(0,()->{
					clawControl.toggleAutoOpenClose();
				})
				.waitSeconds(0.15)
				//moves to
				.UNSTABLE_addTemporalMarkerOffset(0.25,()->{
					clawControl.toggleWristRotate();
					slideControl.setCustom(790);
					armControl.setIntake();//ppcocaine
				})
				.waitSeconds(0.25)
				.strafeLeft(14)
				.lineToLinearHeading(new Pose2d(46 ,25.5, Math.toRadians(angle)))
				.UNSTABLE_addTemporalMarkerOffset(0,()->{
					clawControl.toggleAutoOpenClose();
				})
				.waitSeconds(.25)
				.UNSTABLE_addTemporalMarkerOffset(0,()->{
					slideControl.setCustom(1270);
				})
				.waitSeconds(.25)
				.lineToLinearHeading(new Pose2d(50,-3.5,Math.toRadians(30)))
				.UNSTABLE_addTemporalMarkerOffset(0.45,()->{
					slideControl.setHighJunction(telemetry);
					armControl.setExtake();
					clawControl.toggleWristRotate();
				})
				/*.strafeRight(17)
				.UNSTABLE_addTemporalMarkerOffset(0,()->{
					slideControl.setCustom(1450);
				})
				.waitSeconds(.5)
				.UNSTABLE_addTemporalMarkerOffset(0,()->{
					clawControl.toggleAutoOpenClose();
				})
				.waitSeconds(0.15)
				.UNSTABLE_addTemporalMarkerOffset(0.25,()->{

					clawControl.toggleWristRotate();
					slideControl.setCustom(710);
					armControl.setIntake();//ppcocaine
				})
				.waitSeconds(0.25)
				.strafeLeft(14)
				.lineToLinearHeading(new Pose2d(46 ,24, Math.toRadians(angle)))
				.UNSTABLE_addTemporalMarkerOffset(0,()-> {
					clawControl.toggleAutoOpenClose();
				})
				.waitSeconds(0.25)
				.UNSTABLE_addTemporalMarkerOffset(0,()->{
					slideControl.setCustom(1190);
				})
				.waitSeconds(0.25)
				.lineToLinearHeading(new Pose2d(46.5,-3.75,Math.toRadians(90)))
				.UNSTABLE_addTemporalMarkerOffset(0.45,()-> {
					slideControl.setHighJunction(telemetry);
					armControl.setExtake();
					clawControl.toggleWristRotate();
				})
				.strafeRight(17)
				.UNSTABLE_addTemporalMarkerOffset(0,()->{
					slideControl.setCustom(1450);
				})
				.waitSeconds(0.5)
				.UNSTABLE_addTemporalMarkerOffset(0,()->{
					clawControl.toggleAutoOpenClose();
				})
				.waitSeconds(0.15)
				.UNSTABLE_addTemporalMarkerOffset(0.25,()->{

					clawControl.toggleWristRotate();
					slideControl.setCustom(630);
					armControl.setIntake();//ppcocaine
				})
				.waitSeconds(0.25)
				.strafeLeft(14)
				.lineToLinearHeading(new Pose2d(46 ,24, Math.toRadians(angle)))
				.UNSTABLE_addTemporalMarkerOffset(0,()->{
					clawControl.toggleAutoOpenClose();
				})
				.waitSeconds(.25)
				.UNSTABLE_addTemporalMarkerOffset(0,()->{
					slideControl.setCustom(1110);
				})
				.waitSeconds(.25)
				.lineToLinearHeading(new Pose2d(46.5,-3.75,Math.toRadians(90)))
				.UNSTABLE_addTemporalMarkerOffset(0.45,()->{
					slideControl.setHighJunction(telemetry);
					armControl.setExtake();
					clawControl.toggleWristRotate();
				})
				.strafeRight(17)
				.UNSTABLE_addTemporalMarkerOffset(0,()->{
					slideControl.setCustom(1450);
				})
				.waitSeconds(.5)
				.UNSTABLE_addTemporalMarkerOffset(0,()->{
					clawControl.toggleAutoOpenClose();
				})
				//.splineToConstantHeading(new Vector2d(x,y),Math.toRadians(angle))

				 */
				.build();

		TrajectorySequence goToConeStack = bot.trajectorySequenceBuilder(junction.end())
				.strafeLeft(14)
				.lineToLinearHeading(new Pose2d(46 ,24, Math.toRadians(angle)))
				.waitSeconds(1)
				.build();











		TrajectorySequence conesToJunction = bot.trajectorySequenceBuilder(goToConeStack.end())
				.lineToLinearHeading(new Pose2d(47,-4.5,Math.toRadians(90)))
				.strafeRight(13)
				//.lineToLinearHeading(new Pose2d(60,-4.5,Math.toRadians(90)))
				.build();

		waitForStart();


		bot.followTrajectorySequenceAsync(junction);
		//bot.followTrajectorySequenceAsync(goToConeStack);
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
