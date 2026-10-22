package org.firstinspires.ftc.teamcode.dcs15815.BuzzBot;

//import com.acmerobotics.roadrunner.geometry.Pose2d;
import static com.qualcomm.robotcore.hardware.HardwareDevice.Manufacturer.GoBilda;

import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.dcs15815.DefenderFramework.DefenderBot.DefenderBotConfiguration;
import org.firstinspires.ftc.teamcode.dcs15815.DefenderFramework.DefenderBot.DefenderBotPosition;
import org.firstinspires.ftc.teamcode.dcs15815.DefenderFramework.DefenderUtilities.DefenderPresets;

public class BuzzBotConfiguration extends DefenderBotConfiguration {

	/* DRIVETRAIN -------------------------------------------------- */

	public static String DRIVETRAIN_MOTOR_BACK_LEFT_NAME = "motor_back_left"; //
	public static String DRIVETRAIN_MOTOR_FRONT_LEFT_NAME = "motor_front_left"; //
	public static String DRIVETRAIN_MOTOR_FRONT_RIGHT_NAME = "motor_front_right"; //
	public static String DRIVETRAIN_MOTOR_BACK_RIGHT_NAME = "motor_back_right"; //

	public static DcMotorSimple.Direction DRIVETRAIN_MOTOR_BACK_LEFT_DIRECTION = DcMotorSimple.Direction.FORWARD;
	public static DcMotorSimple.Direction DRIVETRAIN_MOTOR_FRONT_LEFT_DIRECTION = DcMotorSimple.Direction.FORWARD;
	public static DcMotorSimple.Direction DRIVETRAIN_MOTOR_FRONT_RIGHT_DIRECTION = DcMotorSimple.Direction.REVERSE;
	public static DcMotorSimple.Direction DRIVETRAIN_MOTOR_BACK_RIGHT_DIRECTION = DcMotorSimple.Direction.REVERSE;

	public static double DRIVETRAIN_POWER_MAX = 1;
	public static int DRIVETRAIN_MAX_TICKS_PER_SECOND = 2800;


	/* SYSTEM 1 ---------------------------------------------------------- */


	/* SYSTEM 2 ---------------------------------------------------------- */

	/* SYSTEM 3 ---------------------------------------------------------- */

	/* GAMEPADS -------------------------------------------------------- */

	public static double GAMEPAD1_RIGHT_STICK_X_CURVE = 1;
//	public static double GAMEPAD1_RIGHT_STICK_X_CURVE = 4;
	public static double GAMEPAD1_RIGHT_STICK_X_MAX = 1;


	/* EFFECTS -------------------------------------------------------- */

	public static String EFFECTS_LEDS_NAME = "effects_leds"; // Control Hub, Servo 5


	/* NAVIGATION -------------------------------------------------------- */

	public static double NAVIGATION_TOLERANCE = 1.0;

	public static String NAVIGATION_PINPOINT_NAME = "pinpoint";
	public static double NAVIGATION_PINPOINT_OFFSET_X = 87.5;
	public static double NAVIGATION_PINPOINT_OFFSET_Y = -135.0;
	public static DistanceUnit NAVIGATION_PINPOINT_OFFSET_DISTANCEUNIT = DistanceUnit.MM;
	public static GoBildaPinpointDriver.EncoderDirection NAVIGATION_PINPOINT_ENCODERDIRECTION_X = GoBildaPinpointDriver.EncoderDirection.FORWARD;
	public static GoBildaPinpointDriver.EncoderDirection NAVIGATION_PINPOINT_ENCODERDIRECTION_Y = GoBildaPinpointDriver.EncoderDirection.FORWARD;
	public static GoBildaPinpointDriver.GoBildaOdometryPods NAVIGATION_POD_TYPE = GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD;

	public static double NAVIGATION_ANGULAR_SCALE = 0.997;
	public static double NAVIGATION_LINEAR_SCALE = 0.978;

	public static double NAVIGATION_TOLERANCE_R = 1.0;
	public static double NAVIGATION_TOLERANCE_Y = 7.0; // in degrees
	public static double NAVIGATION_TOLERANCE_X = 1.0;

	public static double NAVIGATION_X_KP = 0.2;
	public static double NAVIGATION_X_KI = 0.4;
	public static double NAVIGATION_X_KD = 0.03;

	public static double NAVIGATION_Y_KP = 0.5;
	public static double NAVIGATION_Y_KI = 4.5;
	public static double NAVIGATION_Y_KD = 0.12;

	public static double NAVIGATION_R_KP = 0.2;
	public static double NAVIGATION_R_KI = 1.0;
	public static double NAVIGATION_R_KD = 0.003;

	public static double NAVIGATION_X_MAXPOWER = 0.6;
	public static double NAVIGATION_Y_MAXPOWER = 0.3;
	public static double NAVIGATION_R_MAXPOWER = 0.3;

	public static long NAVIGATION_SETTLING_TIME = 400; //was 500

	/* PRESETS -------------------------------------------------------- */

	/* ALL AUTONOMOUS -------------------------------------------------------- */
	public static double DRIVETRAIN_POWER_MAX_AUTONOMOUS = 1;



	/* METHODS -------------------------------------------------------- */

	public BuzzBotConfiguration() {
		super();
	}

}