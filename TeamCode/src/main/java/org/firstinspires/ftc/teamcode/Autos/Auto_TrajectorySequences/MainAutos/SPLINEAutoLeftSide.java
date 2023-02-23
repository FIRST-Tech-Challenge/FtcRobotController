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
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Config
@Autonomous(name = "testAuto")
public class SPLINEAutoLeftSide extends PowerPlay_AprilTagDetectionDeposit{
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
	public static double openingX = 37.7;
	public static double openingY = -1;

	// [MEDIUM JUNCTION]
	public static double mediumX = 44.12;
	public static double mediumY = -4.85;

	// [CONE STACK]
	public static double xConeStack = 48.7;
	public static double yConeStack = 26.85;
	public static double coneStackHeading = 89;
	public static double coneStackForward = 8.7;

	// [OPENING MOVE --> MEDIUM JUNCTION]
	public static double openingHeading = 90;
	// [1] MEDIUM JUNCTION --> CONE STACK
	public static double coneStackHeading1 = 93;
	// [1] CONE STACK --> MEDIUM JUNCTION
	public static double mediumHeading1 = 210; // no end tangent for splineTo

	// [2] MEDIUM JUNCTION --> CONE STACK
	public static double coneStackHeading2 = 45;
	// [2] CONE STACK --> MEDIUM JUNCTION
	public static double mediumHeading2 = 45;

	// [3] MEDIUM JUNCTION --> CONE STACK
	public static double coneStackHeading3 = 45;
	// [3] CONE STACK --> MEDIUM JUNCTION
	public static double mediumHeading3 = 45;

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
					slideControl.setCustom(680);
				})
				//.splineTo(new Vector2d(xConeStack, yConeStack), Math.toRadians(coneStackHeading1))
				//.strafeRight(openingStrafe)

				//.splineToLinearHeading(new Pose2d(xConeStack, yConeStack, Math.toRadians(coneStackHeading1)), Math.toRadians(endTangent1))
				//.forward(coneStackForward)

				.lineToLinearHeading(new Pose2d(xConeStack,-1,Math.toRadians(coneStackHeading)))
				.forward(yConeStack)
				.UNSTABLE_addTemporalMarkerOffset(0,()->{
					clawControl.toggleAutoOpenClose();
				})
				.waitSeconds(0.2)
				.UNSTABLE_addTemporalMarkerOffset(0,()->{
					slideControl.setCustom(1200);
				})
				.waitSeconds(0.5)

				.UNSTABLE_addTemporalMarkerOffset(0.5,()->{
					slideControl.setMidJunction();
					armControl.setExtake();
					clawControl.toggleWristRotate();
				})
				.setReversed(true)
				.splineTo(new Vector2d(mediumX, mediumY), Math.toRadians(mediumHeading1))
				.setReversed(false)
				.UNSTABLE_addTemporalMarkerOffset(0,()->{
					slideControl.setIntakeOrGround();
				})
				.waitSeconds(0.2)
				.UNSTABLE_addTemporalMarkerOffset(0,()->{
					clawControl.toggleAutoOpenClose();
				})
				.waitSeconds(0.2)

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