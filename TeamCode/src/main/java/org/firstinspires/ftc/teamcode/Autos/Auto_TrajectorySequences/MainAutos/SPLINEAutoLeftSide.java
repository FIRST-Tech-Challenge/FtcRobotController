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


@Autonomous(name = "SPLINEAutoLeftSide")
@Config
public class SPLINEAutoLeftSide extends PowerPlay_AprilTagDetectionDeposit {

	private Arm armControl;
	private Slide slideControl;
	private Claw clawControl;

	public static double angle = 150;
	public static double angleConeStack = 95;
	public static double xConeStack = 51.5; // TODO: Change this value before forward() command
	public static double yConeStack = 29.5;
	public static int armPosition = 935;
	public static double firstStrafe = 17;

	////////////// FIRST \\\\\\\\\\\\\\\\\\\\\\
	public static double xFirstLinear = 65; // needs to change for medium junction
	public static double yFirstLinear = 2.5; // needs to change for medium junction
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
		//If 13.53 V, y = -3.5
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
		initialize();

		TrajectorySequence junction = bot.trajectorySequenceBuilder(startPose)
				.UNSTABLE_addTemporalMarkerOffset(0,()->{
					clawControl = new Claw(hardwareMap);
					OdoPod odoControl = new OdoPod(hardwareMap);
				})
				.waitSeconds(0.5)
				// Mechanisms deposit preload at medium junction
				.UNSTABLE_addTemporalMarkerOffset(0.75,()->{
					slideControl.setMidJunction();
					armControl.setExtake();
					clawControl.toggleWristRotate();
				})
				// MOVES TO MEDIUM JUNCTION FOR FIRST TIME
				.lineToLinearHeading(new Pose2d(xFirstLinear, yFirstLinear, Math.toRadians(angleFirstLinear)))
				.waitSeconds(0.5)
				.UNSTABLE_addTemporalMarkerOffset(0,()->{
					slideControl.setCustom(750); // Smacks down into the Medium junction
				})
				.waitSeconds(.35)
					// DEPOSIT AT MEDIUM JUNCTION
					.UNSTABLE_addTemporalMarkerOffset(0,()->{
						clawControl.toggleAutoOpenClose();
					})
					.waitSeconds(0.5)
					.UNSTABLE_addTemporalMarkerOffset(0, ()->{
						armControl.setIntake();
						clawControl.toggleWristRotate();
						slideControl.setCustom(1310); // TODO: change this value
					})
				.waitSeconds(0.15)

				// MEDIUM JUNCTION TO CONE STACK
				.splineToLinearHeading(new Pose2d(xConeStack, yConeStack, Math.toRadians(90)), Math.toRadians(0))
				                               // (x position, y position, angle of the bot), shape of the spline)
				.forward(2) // TODO: change this value
				.waitSeconds(0.5)
					.UNSTABLE_addTemporalMarkerOffset(0, () ->{
						clawControl.toggleAutoOpenClose();
					})
				.waitSeconds(0.5)
					.UNSTABLE_addTemporalMarkerOffset(0, () ->{
						slideControl.setCustom(1400); // TODO: change this value
					})
				.waitSeconds(0.5)

				// CONE STACK TO MEDIUM JUNCTION
				.splineToLinearHeading(new Pose2d(xFirstLinear, yFirstLinear, Math.toRadians(angleFirstLinear)), Math.toRadians(90)) // TODO: change this value
				.waitSeconds(2)






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