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
public class SPLINEAutoLeftSide extends PowerPlay_AprilTagDetectionDeposit {
	// [ARM]
		private Arm armControl;
		public static int armPosition = 0;

	// [SLIDES]
		private Slide slideControl;

	// [CLAW]
	    private Claw clawControl;

	// [OPENING MOVE]
	public static double forwardAmount = -50;

	// [MEDIUM JUNCTION]
	public static double mediumX = 45;
	public static double mediumY = -4;
		// [OPENING MOVE --> MEDIUM JUNCTION]
		public static double mediumHeading1 = 52;
		// [CONE STACK --> MEDIUM JUNCTION]
		public static double mediumHeading2 = 210; // no end tangent for splineTo

	// [CONE STACK]
	public static double xConeStack = 45;
	public static double yConeStack = 15;
	public static double coneStackForward = 8.7;
		// [1] MEDIUM JUNCTION --> CONE STACK
		public static double coneStackHeading1 = 93;
		// [2] MEDIUM JUNCTION --> CONE STACK
		public static double coneStackHeading2 = 45;

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

		TrajectorySequence junction = bot.trajectorySequenceBuilder(startPose)
				/*
				.UNSTABLE_addTemporalMarkerOffset(0,()->{
					clawControl = new Claw(hardwareMap);
					OdoPod odoControl = new OdoPod(hardwareMap);
				})
				 */ // claw close
				.waitSeconds(0.5)
				.forward(forwardAmount)
				.waitSeconds(0.5)
				.lineToLinearHeading(new Pose2d(mediumX,mediumY,Math.toRadians(mediumHeading1)))
				.waitSeconds(1)
				.splineTo(new Vector2d(xConeStack, yConeStack), Math.toRadians(coneStackHeading1))
				//.splineToSplineHeading(new Pose2d(xConeStack, yConeStack, Math.toRadians(endTurn1)), Math.toRadians(endTangent1))
				.waitSeconds(0.7)
				.forward(coneStackForward)
				.waitSeconds(1)
				.setReversed(true)
				.splineTo(new Vector2d(mediumX, mediumY), Math.toRadians(mediumHeading2))
				.setReversed(false)
				//.splineToSplineHeading(new Pose2d(mediumX, mediumY, Math.toRadians(mediumHeading2)), Math.toRadians(mediumEndTangent2))
				.waitSeconds(1)
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