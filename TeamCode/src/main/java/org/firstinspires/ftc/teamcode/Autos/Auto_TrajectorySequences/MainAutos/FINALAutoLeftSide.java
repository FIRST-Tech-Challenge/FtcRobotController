package org.firstinspires.ftc.teamcode.Autos.Auto_TrajectorySequences.MainAutos;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
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
public class FINALAutoLeftSide extends PowerPlay_AprilTagDetectionDeposit {

	private Arm armControl;
	private Slide slideControl;
	private Claw clawControl;

	public static double angle = 150;
	public static double angleConeStack = 95;
	public static double xConeStack = 51.5;
	public static double yConeStack = 29.5;
	public static int armPosition = 935;
	public static double firstStrafe = 17;

	////////////// FIRST \\\\\\\\\\\\\\\\\\\\\\
	public static double xFirstLinear = 65;
	public static double yFirstLinear = 2.5;
	public static double angleFirstLinear = 85;

	////////////// SECOND \\\\\\\\\\\\\\\\\\\\\\
	public static double xSecondToJunction = 53;
	public static double ySecondToJunction = -4;
	public static double angleSecond = 180;

	////////////// THIRD \\\\\\\\\\\\\\\\\\\\\\
	public static double xThirdToJunction = 54;
	public static double yThirdToJunction = -4;
	public static double cycleThreeAngle = 140;
	public static double angleThird = 180;

	////////////// FOURTH \\\\\\\\\\\\\\\\\\\\\\
	public static double xFourthToJunction = 53;
	public static double yFourthToJunction = -3;

	////////////// PARK \\\\\\\\\\\\\\\\\\\\\\
	public static double xParkZoneOne=48.5;
	public static double yParkZoneOne=26;

	public static double xSecondPark = 54;
	public static double ySecondPark = 0;

	public static double xParkZoneThree=-46;
	public static double yParkZoneThree=-25;
	public static int strafeThird = -28;

	public void initialize(){
		armControl = new Arm(hardwareMap);
		slideControl = new Slide(hardwareMap);
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
				.UNSTABLE_addTemporalMarkerOffset(0,()->{
					clawControl = new Claw(hardwareMap);
					OdoPod odoControl = new OdoPod(hardwareMap);
				})
				.waitSeconds(0.5)
				//If 13.53 V, y = -3.5
				// Mechanisms deposit preload at high junction with a 0.65 delay
				.UNSTABLE_addTemporalMarkerOffset(0.75,()->{
					slideControl.setCustom(2200);
					armControl.setExtake();
					clawControl.toggleWristRotate();

				})
				// MOVES TO HIGH JUNCTION FOR FIRST TIME
				.lineToLinearHeading(new Pose2d(xFirstLinear, yFirstLinear, Math.toRadians(angleFirstLinear)))
				.waitSeconds(0.5)
				.UNSTABLE_addTemporalMarkerOffset(0,()->{
					slideControl.setCustom(1370); // Overrides current target position of slides
				})
				.waitSeconds(.35)
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
					slideControl.setCustom(740);
					armControl.setIntake();
				})
				.waitSeconds(0.25)
				.strafeLeft(firstStrafe)
				.forward(28)
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
					armControl.setCustom(armPosition);
					clawControl.toggleWristRotate();
				})

				.lineToLinearHeading(new Pose2d(xSecondToJunction,ySecondToJunction,Math.toRadians(angle)))
				.UNSTABLE_addTemporalMarkerOffset(0.15,()->{
					slideControl.setCustom(1370);
				})
				.waitSeconds(0.35) // OLD VALUE: 0.25
				.UNSTABLE_addTemporalMarkerOffset(0,()->{
					clawControl.toggleAutoOpenClose();
				})

				.waitSeconds(0.25) // OLD VALUE: 0.15
				// PREPPING PIDS FOR CONE STACK \\
				.UNSTABLE_addTemporalMarkerOffset(0.25,()->{ // OLD VALUE: 0.25
					clawControl.toggleWristRotate();
					slideControl.setCustom(630);// OLD VALUE: 620
					armControl.setIntake();
				})
				.waitSeconds(0.35) // OLD VALUE: 0.25
				// GOING TO THE CONE STACK \\
				.lineToLinearHeading(new Pose2d(xConeStack ,0, Math.toRadians(angleConeStack)))
				.forward(yConeStack)
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
					armControl.setCustom(950);
					clawControl.toggleWristRotate();
				})
				.lineToLinearHeading(new Pose2d(xThirdToJunction,yThirdToJunction,Math.toRadians(angle)))

				.UNSTABLE_addTemporalMarkerOffset(0,()->{
					slideControl.setCustom(1370);
				})
				.waitSeconds(.6)
				.UNSTABLE_addTemporalMarkerOffset(0,()->{
					clawControl.toggleAutoOpenClose();
				})

				.waitSeconds(0.15)
				.UNSTABLE_addTemporalMarkerOffset(0.25,()->{

					clawControl.toggleWristRotate();
					slideControl.setCustom(490);
					armControl.setIntake();
				})
				.waitSeconds(0.25)
				.lineToLinearHeading(new Pose2d(xConeStack ,0, Math.toRadians(angleConeStack)))
				.forward(yConeStack)
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
					armControl.setCustom(950);
					clawControl.toggleWristRotate();
				})
				.waitSeconds(0.5)

				.lineToLinearHeading(new Pose2d(xFourthToJunction,yFourthToJunction,Math.toRadians(cycleThreeAngle)))

				.UNSTABLE_addTemporalMarkerOffset(0,()->{
					slideControl.setCustom(1370);
				})
				.waitSeconds(.5)
				.UNSTABLE_addTemporalMarkerOffset(0,()->{
					clawControl.toggleAutoOpenClose();
				})
				.waitSeconds(0.15)
				.UNSTABLE_addTemporalMarkerOffset(0,()->{
					slideControl.setIntakeOrGround();
					armControl.setIntake();
				})
				.waitSeconds(0.25)
				.UNSTABLE_addTemporalMarkerOffset(0,()->{

					clawControl.toggleWristRotate();

				})
				.waitSeconds(0.25)



				.addTemporalMarker(() -> {
					if(tagUse == 1){
						TrajectorySequence zoneOne = bot.trajectorySequenceBuilder(new Pose2d(xFourthToJunction,yFourthToJunction,Math.toRadians(angle)))
								.lineToLinearHeading(new Pose2d(48.5 ,0, Math.toRadians(91.45)))
								.forward(28)
								.build();
						bot.followTrajectorySequenceAsync(zoneOne);
					}else if(tagUse == 2) {
						TrajectorySequence zoneTwo = bot.trajectorySequenceBuilder(new Pose2d(xFourthToJunction,yFourthToJunction,Math.toRadians(angle)))
								.lineToLinearHeading(new Pose2d(48.5 ,0, Math.toRadians(angleSecond)))
								.build();
						bot.followTrajectorySequenceAsync(zoneTwo);
					}else{
						Trajectory zoneThree = bot.trajectoryBuilder(new Pose2d(xFourthToJunction,yFourthToJunction,Math.toRadians(angle)))
								.lineToLinearHeading(new Pose2d(48.5 ,-23, Math.toRadians(angleThird)))
								.build();
						bot.followTrajectoryAsync(zoneThree);
					}
				})
				.waitSeconds(3)
				.build();
		scan();

		waitForStart();
		bot.followTrajectorySequenceAsync(junction);
		//If we move on, how to eat chicken light skin edition will drop

		while(opModeIsActive()  && !isStopRequested()){
			bot.update();
			armControl.update(telemetry);
			telemetry.addLine("run");
			telemetry.update();
			slideControl.update(telemetry);
		}

	}
}