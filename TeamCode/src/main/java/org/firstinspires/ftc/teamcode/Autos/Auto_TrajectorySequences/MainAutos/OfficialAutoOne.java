package org.firstinspires.ftc.teamcode.Autos.Auto_TrajectorySequences.MainAutos;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.MechanismTemplates.Arm;
import org.firstinspires.ftc.teamcode.MechanismTemplates.Claw;
import org.firstinspires.ftc.teamcode.MechanismTemplates.OdoPod;
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

	public static double xFirstLinear = 63;
	public static double yFirstLinear = 2;
	public static double angle = 147;
	public static double xSecondToJunction = 53;
	public static double ySecondToJunction = -4;
	public static double xThirdToJunction = 54;
	public static double yThirdToJunction = -4;
	public static double xFourthToJunction = 54;
	public static double yFourthToJunction = -4;
	public static double xParkZoneOne=46;
	public static double yParkZoneOne=22;
	public static double xParkZoneThree=46;
	public static double yParkZoneThree=-25;
	public static double angleConeStack = 91.45;


	public void initialize(){
		armControl = new Arm(hardwareMap);
		slideControl = new Slide(hardwareMap);
		clawControl = new Claw(hardwareMap);
		OdoPod odoControl = new OdoPod(hardwareMap);
	}

	public void scan(){
		super.runOpMode();
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
				// Mechanisms deposit preload at high junction with a 0.65 delay
				.UNSTABLE_addTemporalMarkerOffset(0.75,()->{
					slideControl.setCustom(2200);
					armControl.setExtake();
					clawControl.toggleWristRotate();

				})
				// MOVES TO HIGH JUNCTION FOR FIRST TIME
				.lineToLinearHeading(new Pose2d(xFirstLinear, yFirstLinear, Math.toRadians(90)))
				.waitSeconds(0.5)
				.UNSTABLE_addTemporalMarkerOffset(0,()->{
					slideControl.setCustom(1350); // Overrides current target position of slides
				})
				.waitSeconds(.8)
				//opens claw
				.UNSTABLE_addTemporalMarkerOffset(0,()->{
					clawControl.toggleAutoOpenClose();
				})
				.waitSeconds(0.5)
				.UNSTABLE_addTemporalMarkerOffset(0, ()->{
					armControl.setIntake();
					clawControl.toggleWristRotate();
				})
				.waitSeconds(0.15)
				// MOVES TO CONE STACK
				.UNSTABLE_addTemporalMarkerOffset(0.25,()->{
					slideControl.setCustom(710);
					armControl.setIntake();
				})
				.waitSeconds(0.25)
				.strafeLeft(13)
				.forward(27)
				.UNSTABLE_addTemporalMarkerOffset(0,()->{
					clawControl.toggleAutoOpenClose();
				})






				// GOING TO HIGH JUNCTION FOR THE SECOND TIME
				.waitSeconds(.25)
				.UNSTABLE_addTemporalMarkerOffset(0,()->{
					slideControl.setCustom(1370);
				})
				.waitSeconds(.45)
				.UNSTABLE_addTemporalMarkerOffset(0.7,()->{ //0.7 old value
					slideControl.setCustom(2200); // 2nd time at high junction
					armControl.setCustom(990);
					clawControl.toggleWristRotate();
				})

				.lineToLinearHeading(new Pose2d(xSecondToJunction,ySecondToJunction,Math.toRadians(angle)))
				.UNSTABLE_addTemporalMarkerOffset(1,()->{
					slideControl.setCustom(1350);
				})
				.waitSeconds(1) // OLD VALUE: 0.25
				.UNSTABLE_addTemporalMarkerOffset(0,()->{
					clawControl.toggleAutoOpenClose();
				})

				.waitSeconds(0.25) // OLD VALUE: 0.15
				// PREPPING PIDS FOR CONE STACK \\
				.UNSTABLE_addTemporalMarkerOffset(0.25,()->{ // OLD VALUE: 0.25
					clawControl.toggleWristRotate();
					slideControl.setCustom(600);// OLD VALUE: 620
					armControl.setIntake();
				})
				.waitSeconds(1.25) // OLD VALUE: 0.25
				// GOING TO THE CONE STACK \\
				.lineToLinearHeading(new Pose2d(48.5 ,0, Math.toRadians(91.45)))
				.forward(28)
				.UNSTABLE_addTemporalMarkerOffset(0,()->{
					clawControl.toggleAutoOpenClose();
				})
				.waitSeconds(.25)
				.UNSTABLE_addTemporalMarkerOffset(0,()->{
					slideControl.setCustom(1310); // RAISES SLIDES BEFORE GOING TO THE JUNCTION FOR THE THIRD TIME
				})



				//GOING TO JUNCTION FOR THIRD TIME
				.waitSeconds(.25)
				.UNSTABLE_addTemporalMarkerOffset(0.3,()->{
					slideControl.setCustom(2200);
					armControl.setCustom(990);
					clawControl.toggleWristRotate();
				})
				.lineToLinearHeading(new Pose2d(xThirdToJunction,yThirdToJunction,Math.toRadians(angle)))

				.UNSTABLE_addTemporalMarkerOffset(0.3,()->{
					slideControl.setCustom(1450);
				})
				.waitSeconds(.35)
				.UNSTABLE_addTemporalMarkerOffset(0,()->{
					clawControl.toggleAutoOpenClose();
				})

				.waitSeconds(0.15)
				.UNSTABLE_addTemporalMarkerOffset(0.25,()->{

					clawControl.toggleWristRotate();
					slideControl.setCustom(500);
					armControl.setIntake();
				})
				.waitSeconds(0.25)

				.lineToLinearHeading(new Pose2d(48.5 ,0, Math.toRadians(91.45)))
				.forward(28)
				.UNSTABLE_addTemporalMarkerOffset(0,()->{
					clawControl.toggleAutoOpenClose();
				})
				.waitSeconds(.25)
				.UNSTABLE_addTemporalMarkerOffset(0,()->{
					slideControl.setCustom(1310);
				})
				.waitSeconds(.25)
				.UNSTABLE_addTemporalMarkerOffset(0.7,()->{
					slideControl.setCustom(2200);
					armControl.setCustom(990);
					clawControl.toggleWristRotate();
				})
				.waitSeconds(0.5)
				.lineToLinearHeading(new Pose2d(xFourthToJunction,yFourthToJunction,Math.toRadians(angle)))

				.UNSTABLE_addTemporalMarkerOffset(1,()->{
					slideControl.setCustom(1350);
				})
				.waitSeconds(.25)
				.UNSTABLE_addTemporalMarkerOffset(0,()->{
					clawControl.toggleAutoOpenClose();
				})
				.waitSeconds(0.15)
				.UNSTABLE_addTemporalMarkerOffset(0.25,()->{

					clawControl.toggleWristRotate();
					slideControl.setIntakeOrGround();
					armControl.setIntake();
				})

				.waitSeconds(0.25)
				/*
				.addTemporalMarker(() -> {
					if(tagUse == 1){
						Trajectory zoneOne = bot.trajectoryBuilder(new Pose2d(xFourthToJunction,yFourthToJunction,Math.toRadians(angle)))
								.lineToLinearHeading(new Pose2d(xParkZoneOne,yParkZoneOne,91))
								.build();
						bot.followTrajectoryAsync(zoneOne);
					}else if(tagUse == 3){
						Trajectory zoneThree = bot.trajectoryBuilder(new Pose2d(xFourthToJunction,yFourthToJunction,Math.toRadians(angle)))
								.lineToLinearHeading(new Pose2d(xParkZoneThree,yParkZoneThree,90))
								.build();
						bot.followTrajectoryAsync(zoneThree);
					}
				})
				*/

				.build();

		scan();

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