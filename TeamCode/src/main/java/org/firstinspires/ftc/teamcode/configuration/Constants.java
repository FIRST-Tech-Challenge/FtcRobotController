	package org.firstinspires.ftc.teamcode.configuration;


	import com.pedropathing.control.FilteredPIDFCoefficients;
	import com.pedropathing.control.PIDFCoefficients;
	import com.pedropathing.follower.Follower;
	import com.pedropathing.follower.FollowerConstants;
	import com.pedropathing.ftc.FollowerBuilder;
	import com.pedropathing.ftc.drivetrains.MecanumConstants;
	import com.pedropathing.ftc.localization.constants.PinpointConstants;
	import com.pedropathing.paths.PathConstraints;
	import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
	import com.qualcomm.robotcore.hardware.DcMotorSimple;
	import com.qualcomm.robotcore.hardware.HardwareMap;

	import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

	public class Constants {


		/* ===================== PEDRO ===================== */


		/* ===================== FTC SDK ===================== */

		/* ===================== FOLLOWER ===================== */
		public static final FollowerConstants FOLLOWER_CONSTANTS =
				new FollowerConstants()
						.mass(10.98)
						.forwardZeroPowerAcceleration(-48.78168744585494)
						.lateralZeroPowerAcceleration(-60.323478035475866)

						.translationalPIDFCoefficients(new PIDFCoefficients(0.09, 0.000001, 0.015, 0.025))
						.secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(0.16, 0.00001, 0.01, 0.02))
						.useSecondaryTranslationalPIDF(true)

						.headingPIDFCoefficients(new PIDFCoefficients(0.9, 0.0001, 0.015, 0.023))
						.secondaryHeadingPIDFCoefficients(new PIDFCoefficients(1.45, 0.001, 0.015, 0.02))
						.useSecondaryHeadingPIDF(true)

						.drivePIDFCoefficients(new FilteredPIDFCoefficients(0.05, 0, 0.0005, 0.6, 0.01))
						.secondaryDrivePIDFCoefficients(new FilteredPIDFCoefficients(0.03, 0, 0.0005, 0.6, 0.01))
						.useSecondaryDrivePIDF(true); // Robot mass in kg

		public static final PathConstraints PATH_CONSTRAINTS =
				new PathConstraints(
						0.995,
						0.1,
						0.1,
						0.009,
						50,
						1.25,
						10,
						0.65
				);
		/* ===================== DRIVETRAIN ===================== */
		public static final MecanumConstants DRIVE_CONSTANTS =
				new MecanumConstants()
						.maxPower(1.0)
						.leftFrontMotorName("frontleft")
						.leftRearMotorName("backleft")
						.rightFrontMotorName("frontright")
						.rightRearMotorName("backright")
						.leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
						.leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
						.rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
						.rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)

						.xVelocity(72.36733888641119)
						.yVelocity(60.81596266378568)
						.useBrakeModeInTeleOp(true);
		public static PinpointConstants pinpointConstants = new PinpointConstants()
				.forwardPodY(0)
				.strafePodX(-4)
				.distanceUnit(DistanceUnit.INCH)
				.hardwareMapName(Settings.Hardware.PINPOINT)
				.encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
				.forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
				.strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);

		/* ===================== FOLLOWER BUILDER ===================== */
		public static Follower createFollower(HardwareMap hardwareMap) {
			return new FollowerBuilder(FOLLOWER_CONSTANTS, hardwareMap)
					.pathConstraints(PATH_CONSTRAINTS)
					.mecanumDrivetrain(DRIVE_CONSTANTS)
					.pinpointLocalizer(pinpointConstants)
					.build();
		}
	}

