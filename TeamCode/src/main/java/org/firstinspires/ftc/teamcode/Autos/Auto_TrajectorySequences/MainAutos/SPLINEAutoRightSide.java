package org.firstinspires.ftc.teamcode.Autos.Auto_TrajectorySequences.MainAutos;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.MechanismTemplates.Arm;
import org.firstinspires.ftc.teamcode.MechanismTemplates.Claw;
import org.firstinspires.ftc.teamcode.MechanismTemplates.OdoPod;
import org.firstinspires.ftc.teamcode.MechanismTemplates.Slide;
import org.firstinspires.ftc.teamcode.TeleOps.AprilTags.PowerPlay_AprilTagDetectionDeposit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Config
@Autonomous(name = "testAuto2")
public class SPLINEAutoRightSide extends PowerPlay_AprilTagDetectionDeposit{
	public static double endTangent1 = 40;
	public static double openingStrafe = 8;
	// [ARM]
	private Arm armControl;
	public static int armPosition = 0;

	// [SLIDES]
	private Slide slideControl;

	// [CLAW]
	private Claw clawControl;

	// [OPENING MOVE]
	public static double openingX = 35;
	public static double openingY = 1;

	// [MEDIUM JUNCTION]
	public static double mediumX1 = 43.5;
	public static double mediumY1 = 5;
	public static double mediumX2 = 43.5;
	public static double mediumY2 = 5;
	public static double mediumX3 = 43.5;
	public static double mediumY3 = 5;
	public static double mediumX4 = 43.5;
	public static double mediumY4 = 5;
	public static double mediumX5 = 43.5;
	public static double mediumY5 = 5;


	// [CONE STACK]
	public static double xConeStack1 = 48.7;
	public static double yConeStack = 26.85;
	public static double xConeStack2 = 48.7;
	public static double xConeStack3 = 48.7;
	public static double xConeStack4 = 48.7;
	public static double xConeStack5 = 48.7;
	public static double coneStackHeading = 269;
	public static double coneStackForward = 8.7;

	// [OPENING MOVE --> MEDIUM JUNCTION]
	public static double openingHeading = 270;
	// [1] MEDIUM JUNCTION --> CONE STACK
	public static double coneStackHeading1 = 267;
	// [1] CONE STACK --> MEDIUM JUNCTION
	public static double mediumHeading1 = 145; // no end tangent for splineTo

	// [2] MEDIUM JUNCTION --> CONE STACK
	public static double coneStackHeading2 = 45;
	// [2] CONE STACK --> MEDIUM JUNCTION
	public static double mediumHeading2 = 145;

	// [3] MEDIUM JUNCTION --> CONE STACK
	public static double coneStackHeading3 = 45;
	// [3] CONE STACK --> MEDIUM JUNCTION
	public static double mediumHeading3 = 145;

	// [4] MEDIUM JUNCTION --> CONE STACK
	public static double coneStackHeading4 = 45;
	// [4] CONE STACK --> MEDIUM JUNCTION
	public static double mediumHeading4 = 145;

	public void initialize(){
		armControl = new Arm(hardwareMap);
		slideControl = new Slide(hardwareMap);
	}

	public void scan(){
		super.runOpMode();
	}

	@Override
	public void runOpMode() {
		Pose2d startPose = new Pose2d(0, 0, Math.toRadians(180));
		SampleMecanumDrive bot = new SampleMecanumDrive(hardwareMap);
		bot.setPoseEstimate(startPose);
		initialize();

		TrajectorySequence junction = bot.trajectorySequenceBuilder(startPose)
				.UNSTABLE_addTemporalMarkerOffset(0,()->{
					clawControl = new Claw(hardwareMap);
					OdoPod odoControl = new OdoPod(hardwareMap);
				})
				// claw close
				.waitSeconds(0.45)
				.UNSTABLE_addTemporalMarkerOffset(1,()->{
					slideControl.setMidJunction();
					armControl.setExtake();
					clawControl.toggleWristRotate();
				})
				.lineToLinearHeading(new Pose2d(openingX,openingY,Math.toRadians(openingHeading)))
				.UNSTABLE_addTemporalMarkerOffset(0.15,()->{
					slideControl.setIntakeOrGround(); // TODO change this to custom height since we'll be at the conestack height
				})

				.waitSeconds(0.25)
				.UNSTABLE_addTemporalMarkerOffset(0,()->{
					clawControl.toggleAutoOpenClose();
				})
				.waitSeconds(0.25)

				.UNSTABLE_addTemporalMarkerOffset(0,()->{
					armControl.setIntake();
					clawControl.toggleWristRotate();
				})
				.waitSeconds(0.25)
				.UNSTABLE_addTemporalMarkerOffset(0,()->{
					slideControl.setCustom(600);
				})
				//.splineTo(new Vector2d(xConeStack, yConeStack), Math.toRadians(coneStackHeading1))
				//.strafeRight(openingStrafe)

				//.splineToLinearHeading(new Pose2d(xConeStack, yConeStack, Math.toRadians(coneStackHeading1)), Math.toRadians(endTangent1))
				//.forward(coneStackForward)

				.lineToLinearHeading(new Pose2d(xConeStack1,1,Math.toRadians(coneStackHeading)))
				.forward(yConeStack)
				.UNSTABLE_addTemporalMarkerOffset(0,()->{
					clawControl.toggleAutoOpenClose();
				})
				.waitSeconds(0.2)
				.UNSTABLE_addTemporalMarkerOffset(0,()->{
					slideControl.setCustom(1275);
				})
				.waitSeconds(0.5)

				.UNSTABLE_addTemporalMarkerOffset(0.5,()->{
					slideControl.setCustom(900);
					armControl.setAutoExtake();
					clawControl.toggleWristRotate();
				})

				.setReversed(true)
				.splineTo(new Vector2d(mediumX1, mediumY1), Math.toRadians(mediumHeading1))
				.setReversed(false)
				.UNSTABLE_addTemporalMarkerOffset(0,()->{
					slideControl.setIntakeOrGround();
				})
				.waitSeconds(0.2)
				.UNSTABLE_addTemporalMarkerOffset(0,()->{
					clawControl.toggleAutoOpenClose();
				})
				.waitSeconds(0.2)

				.UNSTABLE_addTemporalMarkerOffset(0,()->{
					armControl.setIntake();
					clawControl.toggleWristRotate();
				})
				.waitSeconds(0.25)
				.UNSTABLE_addTemporalMarkerOffset(0,()->{
					slideControl.setCustom(475);
				})
				//.splineTo(new Vector2d(xConeStack, yConeStack), Math.toRadians(coneStackHeading1))
				//.strafeRight(openingStrafe)

				//.splineToLinearHeading(new Pose2d(xConeStack, yConeStack, Math.toRadians(coneStackHeading1)), Math.toRadians(endTangent1))
				//.forward(coneStackForward)

				.lineToLinearHeading(new Pose2d(xConeStack2,1,Math.toRadians(coneStackHeading)))
				.forward(yConeStack)
				.UNSTABLE_addTemporalMarkerOffset(0,()->{
					clawControl.toggleAutoOpenClose();
				})
				.waitSeconds(0.2)
				.UNSTABLE_addTemporalMarkerOffset(0,()->{
					slideControl.setCustom(1275);
				})
				.waitSeconds(0.5)

				.UNSTABLE_addTemporalMarkerOffset(0.5,()->{
					slideControl.setCustom(900);
					armControl.setAutoExtake();
					clawControl.toggleWristRotate();
				})
				.setReversed(true)
				.splineTo(new Vector2d(mediumX2, mediumY2), Math.toRadians(mediumHeading2))
				.setReversed(false)
				.UNSTABLE_addTemporalMarkerOffset(0,()->{
					slideControl.setIntakeOrGround();
				})
				.waitSeconds(0.2)
				.UNSTABLE_addTemporalMarkerOffset(0,()->{
					clawControl.toggleAutoOpenClose();
				})
				.waitSeconds(0.2)
				.UNSTABLE_addTemporalMarkerOffset(0,()->{
					armControl.setIntake();
					clawControl.toggleWristRotate();
				})
				.waitSeconds(0.25)
				.UNSTABLE_addTemporalMarkerOffset(0,()->{
					slideControl.setCustom(350);
				})
				.lineToLinearHeading(new Pose2d(xConeStack3,1,Math.toRadians(coneStackHeading)))
				.forward(yConeStack)
				.UNSTABLE_addTemporalMarkerOffset(0,()->{
					clawControl.toggleAutoOpenClose();
				})
				.waitSeconds(0.2)
				.UNSTABLE_addTemporalMarkerOffset(0,()->{
					slideControl.setCustom(1275);
				})
				.waitSeconds(0.5)

				.UNSTABLE_addTemporalMarkerOffset(0.5,()->{
					slideControl.setCustom(900);
					armControl.setAutoExtake();
					clawControl.toggleWristRotate();
				})
				.setReversed(true)
				.splineTo(new Vector2d(mediumX3, mediumY3), Math.toRadians(mediumHeading3))
				.setReversed(false)
				.UNSTABLE_addTemporalMarkerOffset(0,()->{
					slideControl.setIntakeOrGround();
				})
				.waitSeconds(0.2)
				.UNSTABLE_addTemporalMarkerOffset(0,()->{
					clawControl.toggleAutoOpenClose();
				})
				.waitSeconds(0.2)
				.UNSTABLE_addTemporalMarkerOffset(0,()->{
					armControl.setIntake();
					clawControl.toggleWristRotate();
				})
				.waitSeconds(0.25)
				.UNSTABLE_addTemporalMarkerOffset(0,()->{
					slideControl.setCustom(225);

				})
				.lineToLinearHeading(new Pose2d(xConeStack4,1,Math.toRadians(coneStackHeading)))
				.forward(yConeStack)
				.UNSTABLE_addTemporalMarkerOffset(0,()->{
					clawControl.toggleAutoOpenClose();
				})
				.waitSeconds(0.2)
				.UNSTABLE_addTemporalMarkerOffset(0,()->{
					slideControl.setCustom(1275);
				})
				.waitSeconds(0.5)

				.UNSTABLE_addTemporalMarkerOffset(0.5,()->{
					slideControl.setCustom(900);
					armControl.setAutoExtake();
					clawControl.toggleWristRotate();
				})
				.setReversed(true)
				.splineTo(new Vector2d(mediumX4, mediumY4), Math.toRadians(mediumHeading4))
				.setReversed(false)
				.UNSTABLE_addTemporalMarkerOffset(0,()->{
					slideControl.setIntakeOrGround();
				})
				.waitSeconds(0.2)
				.UNSTABLE_addTemporalMarkerOffset(0,()->{
					clawControl.toggleAutoOpenClose();
				})
				.waitSeconds(0.2)
				.UNSTABLE_addTemporalMarkerOffset(0,()->{
					armControl.setIntake();
					clawControl.toggleWristRotate();
				})
				.waitSeconds(0.25)
				.UNSTABLE_addTemporalMarkerOffset(0,()->{
					slideControl.setIntakeOrGround();
				})
				.waitSeconds(0.1)
				.addTemporalMarker(() -> {
					if(tagUse == 1){
						TrajectorySequence zoneOne = bot.trajectorySequenceBuilder(new Pose2d(mediumX4,mediumY4,Math.toRadians(mediumHeading4)))
								.lineToLinearHeading(new Pose2d(51 ,27, Math.toRadians(271)))
								.build();
						bot.followTrajectorySequenceAsync(zoneOne);
					}else if(tagUse == 2) {
						TrajectorySequence zoneTwo = bot.trajectorySequenceBuilder(new Pose2d(mediumX4,mediumY4,Math.toRadians(mediumHeading4)))
								.lineToLinearHeading(new Pose2d(51 ,0, Math.toRadians(271)))
								.build();
						bot.followTrajectorySequenceAsync(zoneTwo);
					}else{
						Trajectory zoneThree = bot.trajectoryBuilder(new Pose2d(mediumX4,mediumY4,Math.toRadians(mediumHeading4)))
								.lineToLinearHeading(new Pose2d(51 ,-23, Math.toRadians(271)))
								.build();
						bot.followTrajectoryAsync(zoneThree);
					}
				})

				/*

				//.splineToSplineHeading(new Pose2d(mediumX, mediumY, Math.toRadians(mediumHeading2)), Math.toRadians(mediumEndTangent2))
				.waitSeconds(1)
				*/
				.build();
		scan();
		waitForStart();
		bot.followTrajectorySequenceAsync(junction);

		while(opModeIsActive()  && !isStopRequested()){
			bot.update();
			armControl.update(telemetry);
			slideControl.update(telemetry);
		}
	}
}
// EAT CHICKEN ┻━┻ ︵ヽ(`Д´)ﾉ︵ ┻━┻
