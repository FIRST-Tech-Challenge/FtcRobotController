package org.firstinspires.ftc.teamcode.pedro;

import com.bylazar.configurables.PanelsConfigurables;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.configurables.annotations.IgnoreConfigurable;
import com.bylazar.field.FieldManager;
import com.bylazar.field.PanelsField;
import com.bylazar.field.Style;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.*;
import com.pedropathing.math.*;
import com.pedropathing.paths.*;
import com.pedropathing.telemetry.SelectableOpMode;
import com.pedropathing.util.*;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

//import org.firstinspires.ftc.teamcode.Drawing;
import org.firstinspires.ftc.teamcode.configuration.Constants;
import static org.firstinspires.ftc.teamcode.pedro.Tuning.follower;
import static org.firstinspires.ftc.teamcode.pedro.Tuning.telemetryM;

import java.util.ArrayList;
import java.util.List;

/**
 * This is the Tuning class. It contains a selection menu for various tuning OpModes.
 *
 * @author Baron Henderson - 20077 The Indubitables
 * @version 1.0, 6/26/2025
 */
@Configurable
@Disabled // PHASE 1: hidden from the Driver Hub list. Delete this line to use the Pedro tuners.
@TeleOp(name = "Tuning", group = "Pedro Pathing")
public class Tuning extends SelectableOpMode {
	public static Follower follower;

	@IgnoreConfigurable
	static PoseHistory poseHistory;

	@IgnoreConfigurable
	static TelemetryManager telemetryM;

	@IgnoreConfigurable
	static ArrayList<String> changes = new ArrayList<>();

	public Tuning() {
		super("Select a Tuning OpMode", s -> {
			s.folder("Localization", l -> {
				l.add("Localization Test", LocalizationTest::new);
				l.add("Forward Tuner", ForwardTuner::new);
				l.add("Lateral Tuner", LateralTuner::new);
				l.add("Turn Tuner", TurnTuner::new);
			});
			s.folder("Automatic", a -> {
				a.add("Forward Velocity Tuner", ForwardVelocityTuner::new);
				a.add("Lateral Velocity Tuner", LateralVelocityTuner::new);
				a.add("Forward Zero Power Acceleration Tuner", ForwardZeroPowerAccelerationTuner::new);
				a.add("Lateral Zero Power Acceleration Tuner", LateralZeroPowerAccelerationTuner::new);
			});
			s.folder("Manual", p -> {
				p.add("Translational Tuner", TranslationalTuner::new);
				p.add("Heading Tuner", HeadingTuner::new);
				p.add("Drive Tuner", DriveTuner::new);
				p.add("Line Tuner", Line::new);
				p.add("Centripetal Tuner", CentripetalTuner::new);
			});
			s.folder("Tests", p -> {
				p.add("Line", Line::new);
				p.add("Triangle", Triangle::new);
				p.add("Circle", Circle::new);
			});
		});
	}

	@Override
	public void onSelect() {
		if (follower == null) {
			follower = Constants.createFollower(hardwareMap);
			PanelsConfigurables.INSTANCE.refreshClass(this);
		} else {
			follower = Constants.createFollower(hardwareMap);
		}

		follower.setStartingPose(new Pose());
		poseHistory = follower.getPoseHistory();
		telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
		Drawing.init();
	}

	@Override
	public void onLog(List<String> lines) {}

	public static void drawOnlyCurrent() {
		try {
			Drawing.drawRobot(follower.getPose());
			Drawing.sendPacket();
		} catch (Exception e) {
			throw new RuntimeException("Drawing failed " + e);
		}
	}

	public static void draw() {
		Drawing.drawDebug(follower);
	}

	/** This creates a full stop of the robot by setting the drive motors to run at 0 power. */
	public static void stopRobot() {
		follower.startTeleopDrive(true);
		follower.setTeleOpDrive(0, 0, 0, true);
	}
}

/**
 * This is the LocalizationTest OpMode.
 */
class LocalizationTest extends OpMode {

	@Override
	public void init() {
		follower.setStartingPose(new Pose(72, 72));
	}

	@Override
	public void init_loop() {
		Tuning.telemetryM.debug("This will print your robot's position to telemetry while "
				+ "allowing robot control through a basic mecanum drive on gamepad 1.");
		Tuning.telemetryM.update(telemetry);
		Tuning.follower.update();
		Tuning.drawOnlyCurrent();
	}

	@Override
	public void start() {
		Tuning.follower.startTeleopDrive();
		Tuning.follower.update();
	}

	@Override
	public void loop() {
		follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, true);
		follower.update();

		telemetryM.debug("x:" + follower.getPose().getX());
		telemetryM.debug("y:" + follower.getPose().getY());
		telemetryM.debug("heading:" + follower.getPose().getHeading());
		telemetryM.debug("total heading:" + follower.getTotalHeading());
		telemetryM.update(telemetry);

		Tuning.draw();
	}
}

/**
 * This is the ForwardTuner OpMode.
 */
class ForwardTuner extends OpMode {
	public static double DISTANCE = 48;

	@Override
	public void init() {
		follower.setStartingPose(new Pose(72, 72));
		follower.update();
		Tuning.drawOnlyCurrent();
	}

	@Override
	public void init_loop() {
		telemetryM.debug("Pull your robot forward " + DISTANCE + " inches. Your forward ticks to inches will be shown on the telemetry.");
		telemetryM.update(telemetry);
		Tuning.drawOnlyCurrent();
	}

	@Override
	public void loop() {
		follower.update();

		telemetryM.debug("Distance Moved: " + (follower.getPose().getX() - 72));
		telemetryM.debug("The multiplier will display what your forward ticks to inches should be to scale your current distance to " + DISTANCE + " inches.");
		telemetryM.debug("Multiplier: " + (DISTANCE / ((follower.getPose().getX() - 72) / follower.getPoseTracker().getLocalizer().getForwardMultiplier())));
		telemetryM.update(telemetry);

		Tuning.draw();
	}
}

/**
 * This is the LateralTuner OpMode.
 */
class LateralTuner extends OpMode {
	public static double DISTANCE = 48;

	@Override
	public void init() {
		follower.setStartingPose(new Pose(72, 72));
		follower.update();
		Tuning.drawOnlyCurrent();
	}

	@Override
	public void init_loop() {
		telemetryM.debug("Pull your robot to the right " + DISTANCE + " inches. Your strafe ticks to inches will be shown on the telemetry.");
		telemetryM.update(telemetry);
		Tuning.drawOnlyCurrent();
	}

	@Override
	public void loop() {
		follower.update();

		telemetryM.debug("Distance Moved: " + (follower.getPose().getY() - 72));
		telemetryM.debug("The multiplier will display what your strafe ticks to inches should be to scale your current distance to " + DISTANCE + " inches.");
		telemetryM.debug("Multiplier: " + (DISTANCE / ((follower.getPose().getY() - 72) / follower.getPoseTracker().getLocalizer().getLateralMultiplier())));
		telemetryM.update(telemetry);

		Tuning.draw();
	}
}

/**
 * This is the TurnTuner OpMode.
 */
class TurnTuner extends OpMode {
	public static double ANGLE = 2 * Math.PI;

	@Override
	public void init() {
		follower.setStartingPose(new Pose(72, 72));
		follower.update();
		Tuning.drawOnlyCurrent();
	}

	@Override
	public void init_loop() {
		telemetryM.debug("Turn your robot " + ANGLE + " radians. Your turn ticks to inches will be shown on the telemetry.");
		telemetryM.update(telemetry);
		Tuning.drawOnlyCurrent();
	}

	@Override
	public void loop() {
		follower.update();

		telemetryM.debug("Total Angle: " + follower.getTotalHeading());
		telemetryM.debug("The multiplier will display what your turn ticks to inches should be to scale your current angle to " + ANGLE + " radians.");
		telemetryM.debug("Multiplier: " + (ANGLE / (follower.getTotalHeading() / follower.getPoseTracker().getLocalizer().getTurningMultiplier())));
		telemetryM.update(telemetry);

		Tuning.draw();
	}
}

/**
 * This is the ForwardVelocityTuner OpMode.
 */
class ForwardVelocityTuner extends OpMode {
	public static double DISTANCE = 48;
	public static double RECORD_NUMBER = 10;
	private final ArrayList<Double> velocities = new ArrayList<>();
	private boolean end;

	@Override
	public void init() {
		follower.setStartingPose(new Pose(72, 72));
	}

	@Override
	public void init_loop() {
		telemetryM.debug("The robot will run at 1 power until it reaches " + DISTANCE + " inches forward.");
		telemetryM.debug("Make sure you have enough room, since the robot has inertia after cutting power.");
		telemetryM.debug("After running the distance, the robot will cut power from the drivetrain and display the forward velocity.");
		telemetryM.debug("Press B on game pad 1 to stop.");
		telemetryM.debug("pose", follower.getPose());
		telemetryM.update(telemetry);
		follower.update();
		Tuning.drawOnlyCurrent();
	}

	@Override
	public void start() {
		for (int i = 0; i < RECORD_NUMBER; i++) {
			velocities.add(0.0);
		}
		follower.startTeleopDrive(true);
		follower.update();
		end = false;
	}

	@Override
	public void loop() {
		if (gamepad1.bWasPressed()) {
			Tuning.stopRobot();
			try {
				requestOpModeStop();
			} catch (NullPointerException e) {
				// Ignore
			}
		}

		follower.update();
		Tuning.draw();

		if (!end) {
			if (Math.abs(follower.getPose().getX()) > (DISTANCE + 72)) {
				end = true;
				Tuning.stopRobot();
			} else {
				follower.setTeleOpDrive(1, 0, 0, true);
				double currentVelocity = Math.abs(follower.poseTracker.getLocalizer().getVelocity().getX());
				velocities.add(currentVelocity);
				velocities.remove(0);
			}
		} else {
			Tuning.stopRobot();
			double average = 0;
			for (double velocity : velocities) {
				average += velocity;
			}
			average /= velocities.size();
			telemetryM.debug("Forward Velocity: " + average);
			telemetryM.debug("\n");
			telemetryM.debug("Press A to set the Forward Velocity temporarily (while robot remains on).");

			for (int i = 0; i < velocities.size(); i++) {
				telemetry.addData(String.valueOf(i), velocities.get(i));
			}

			telemetryM.update(telemetry);
			telemetry.update();

			if (gamepad1.aWasPressed()) {
				follower.setXVelocity(average);
				String message = "XMovement: " + average;
				Tuning.changes.add(message);
			}
		}
	}
}

/**
 * This is the LateralVelocityTuner OpMode.
 */
class LateralVelocityTuner extends OpMode {
	public static double DISTANCE = 48;
	public static double RECORD_NUMBER = 10;
	private final ArrayList<Double> velocities = new ArrayList<>();
	private boolean end;

	@Override
	public void init() {
		follower.setStartingPose(new Pose(72, 72));
	}

	@Override
	public void init_loop() {
		telemetryM.debug("The robot will run at 1 power until it reaches " + DISTANCE + " inches to the left.");
		telemetryM.debug("Make sure you have enough room, since the robot has inertia after cutting power.");
		telemetryM.debug("After running the distance, the robot will cut power from the drivetrain and display the strafe velocity.");
		telemetryM.debug("Press B on Gamepad 1 to stop.");
		telemetryM.update(telemetry);
		follower.update();
		Tuning.drawOnlyCurrent();
	}

	@Override
	public void start() {
		for (int i = 0; i < RECORD_NUMBER; i++) {
			velocities.add(0.0);
		}
		follower.startTeleopDrive(true);
		follower.update();
	}

	@Override
	public void loop() {
		if (gamepad1.bWasPressed()) {
			Tuning.stopRobot();
			try {
				requestOpModeStop();
			} catch (NullPointerException e) {
				// Ignore
			}
		}

		follower.update();
		Tuning.draw();

		if (!end) {
			if (Math.abs(follower.getPose().getY()) > (DISTANCE + 72)) {
				end = true;
				Tuning.stopRobot();
			} else {
				follower.setTeleOpDrive(0, 1, 0, true);
				double currentVelocity = Math.abs(follower.getVelocity().dot(new Vector(1, Math.PI / 2)));
				velocities.add(currentVelocity);
				velocities.remove(0);
			}
		} else {
			Tuning.stopRobot();
			double average = 0;
			for (double velocity : velocities) {
				average += velocity;
			}
			average /= velocities.size();

			telemetryM.debug("Strafe Velocity: " + average);
			telemetryM.debug("\n");
			telemetryM.debug("Press A to set the Lateral Velocity temporarily (while robot remains on).");
			telemetryM.update(telemetry);

			if (gamepad1.aWasPressed()) {
				follower.setYVelocity(average);
				String message = "YMovement: " + average;
				Tuning.changes.add(message);
			}
		}
	}
}

/**
 * This is the ForwardZeroPowerAccelerationTuner OpMode.
 */
class ForwardZeroPowerAccelerationTuner extends OpMode {
	public static double VELOCITY = 75;
	private final ArrayList<Double> accelerations = new ArrayList<>();
	private double previousVelocity;
	private long previousTimeNano;
	private boolean stopping;
	private boolean end;

	@Override
	public void init() {
		follower.setStartingPose(new Pose(72, 72));
	}

	@Override
	public void init_loop() {
		telemetryM.debug("The robot will run forward until it reaches " + VELOCITY + " inches per second.");
		telemetryM.debug("Then, it will cut power from the drivetrain and roll to a stop.");
		telemetryM.debug("Make sure you have enough room.");
		telemetryM.debug("After stopping, the forward zero power acceleration (natural deceleration) will be displayed.");
		telemetryM.debug("Press B on Gamepad 1 to stop.");
		telemetryM.update(telemetry);
		follower.update();
		Tuning.drawOnlyCurrent();
	}

	@Override
	public void start() {
		follower.startTeleopDrive(false);
		follower.update();
		follower.setTeleOpDrive(1, 0, 0, true);
	}

	@Override
	public void loop() {
		if (gamepad1.bWasPressed()) {
			Tuning.stopRobot();
			try {
				requestOpModeStop();
			} catch (NullPointerException e) {
				// Ignore
			}
		}

		follower.update();
		Tuning.draw();

		Vector heading = new Vector(1.0, follower.getPose().getHeading());
		if (!end) {
			if (!stopping) {
				if (follower.getVelocity().dot(heading) > VELOCITY) {
					previousVelocity = follower.getVelocity().dot(heading);
					previousTimeNano = System.nanoTime();
					stopping = true;
					follower.setTeleOpDrive(0, 0, 0, true);
				}
			} else {
				double currentVelocity = follower.getVelocity().dot(heading);
				accelerations.add((currentVelocity - previousVelocity) / ((System.nanoTime() - previousTimeNano) / Math.pow(10.0, 9)));
				previousVelocity = currentVelocity;
				previousTimeNano = System.nanoTime();
				if (currentVelocity < follower.getConstraints().getVelocityConstraint()) {
					end = true;
				}
			}
		} else {
			double average = 0;
			for (double acceleration : accelerations) {
				average += acceleration;
			}
			average /= accelerations.size();

			telemetryM.debug("Forward Zero Power Acceleration (Deceleration): " + average);
			telemetryM.debug("\n");
			telemetryM.debug("Press A to set the Forward Zero Power Acceleration temporarily (while robot remains on).");
			telemetryM.update(telemetry);

			if (gamepad1.aWasPressed()) {
				follower.getConstants().setForwardZeroPowerAcceleration(average);
				String message = "Forward Zero Power Acceleration: " + average;
				Tuning.changes.add(message);
			}
		}
	}
}

/**
 * This is the LateralZeroPowerAccelerationTuner OpMode.
 */
class LateralZeroPowerAccelerationTuner extends OpMode {
	public static double VELOCITY = 50;
	private final ArrayList<Double> accelerations = new ArrayList<>();
	private double previousVelocity;
	private long previousTimeNano;
	private boolean stopping;
	private boolean end;

	@Override
	public void init() {
		follower.setStartingPose(new Pose(72, 72));
	}

	@Override
	public void init_loop() {
		telemetryM.debug("The robot will run to the left until it reaches " + VELOCITY + " inches per second.");
		telemetryM.debug("Then, it will cut power from the drivetrain and roll to a stop.");
		telemetryM.debug("Make sure you have enough room.");
		telemetryM.debug("After stopping, the lateral zero power acceleration (natural deceleration) will be displayed.");
		telemetryM.debug("Press B on game pad 1 to stop.");
		telemetryM.update(telemetry);
		follower.update();
		Tuning.drawOnlyCurrent();
	}

	@Override
	public void start() {
		follower.startTeleopDrive(false);
		follower.update();
		follower.setTeleOpDrive(0, 1, 0, true);
	}

	@Override
	public void loop() {
		if (gamepad1.bWasPressed()) {
			Tuning.stopRobot();
			try {
				requestOpModeStop();
			} catch (NullPointerException e) {
				// Ignore
			}
		}

		follower.update();
		Tuning.draw();

		Vector heading = new Vector(1.0, follower.getPose().getHeading() - Math.PI / 2);
		if (!end) {
			if (!stopping) {
				if (Math.abs(follower.getVelocity().dot(heading)) > VELOCITY) {
					previousVelocity = Math.abs(follower.getVelocity().dot(heading));
					previousTimeNano = System.nanoTime();
					stopping = true;
					follower.setTeleOpDrive(0, 0, 0, true);
				}
			} else {
				double currentVelocity = Math.abs(follower.getVelocity().dot(heading));
				accelerations.add((currentVelocity - previousVelocity) / ((System.nanoTime() - previousTimeNano) / Math.pow(10.0, 9)));
				previousVelocity = currentVelocity;
				previousTimeNano = System.nanoTime();
				if (currentVelocity < follower.getConstraints().getVelocityConstraint()) {
					end = true;
				}
			}
		} else {
			double average = 0;
			for (double acceleration : accelerations) {
				average += acceleration;
			}
			average /= accelerations.size();

			telemetryM.debug("Lateral Zero Power Acceleration (Deceleration): " + average);
			telemetryM.debug("\n");
			telemetryM.debug("Press A to set the Lateral Zero Power Acceleration temporarily (while robot remains on).");
			telemetryM.update(telemetry);

			if (gamepad1.aWasPressed()) {
				follower.getConstants().setLateralZeroPowerAcceleration(average);
				String message = "Lateral Zero Power Acceleration: " + average;
				Tuning.changes.add(message);
			}
		}
	}
}

/**
 * This is the TranslationalTuner OpMode.
 */
class TranslationalTuner extends OpMode {
	public static double DISTANCE = 40;
	private boolean forward = true;

	private Path forwards;
	private Path backwards;

	@Override
	public void init() {
		follower.setStartingPose(new Pose(72, 72));
	}

	@Override
	public void init_loop() {
		telemetryM.debug("This will activate the translational PIDF(s)");
		telemetryM.debug("The robot will try to stay in place while you push it laterally.");
		telemetryM.debug("You can adjust the PIDF values to tune the robot's translational PIDF(s).");
		telemetryM.update(telemetry);
		follower.update();
		Tuning.drawOnlyCurrent();
	}

	@Override
	public void start() {
		follower.deactivateAllPIDFs();
		follower.activateTranslational();
		forwards = new Path(new BezierLine(new Pose(72, 72), new Pose(DISTANCE + 72, 72)));
		forwards.setConstantHeadingInterpolation(0);
		backwards = new Path(new BezierLine(new Pose(DISTANCE + 72, 72), new Pose(72, 72)));
		backwards.setConstantHeadingInterpolation(0);
		follower.followPath(forwards);
	}

	@Override
	public void loop() {
		follower.update();
		Tuning.draw();

		if (!follower.isBusy()) {
			if (forward) {
				forward = false;
				follower.followPath(backwards);
			} else {
				forward = true;
				follower.followPath(forwards);
			}
		}

		telemetryM.debug("Push the robot laterally to test the Translational PIDF(s).");
		telemetryM.addData("Zero Line", 0);
		telemetryM.addData("Error X", follower.errorCalculator.getTranslationalError().getXComponent());
		telemetryM.addData("Error Y", follower.errorCalculator.getTranslationalError().getYComponent());
		telemetryM.update(telemetry);
	}
}

/**
 * This is the HeadingTuner OpMode.
 */
class HeadingTuner extends OpMode {
	public static double DISTANCE = 40;
	private boolean forward = true;

	private Path forwards;
	private Path backwards;

	@Override
	public void init() {
		follower.setStartingPose(new Pose(72, 72));
	}

	@Override
	public void init_loop() {
		telemetryM.debug("This will activate the heading PIDF(s).");
		telemetryM.debug("The robot will try to stay at a constant heading while you try to turn it.");
		telemetryM.debug("You can adjust the PIDF values to tune the robot's heading PIDF(s).");
		telemetryM.update(telemetry);
		follower.update();
		Tuning.drawOnlyCurrent();
	}

	@Override
	public void start() {
		follower.deactivateAllPIDFs();
		follower.activateHeading();
		forwards = new Path(new BezierLine(new Pose(72, 72), new Pose(DISTANCE + 72, 72)));
		forwards.setConstantHeadingInterpolation(0);
		backwards = new Path(new BezierLine(new Pose(DISTANCE + 72, 72), new Pose(72, 72)));
		backwards.setConstantHeadingInterpolation(0);
		follower.followPath(forwards);
	}

	@Override
	public void loop() {
		follower.update();
		Tuning.draw();

		if (!follower.isBusy()) {
			if (forward) {
				forward = false;
				follower.followPath(backwards);
			} else {
				forward = true;
				follower.followPath(forwards);
			}
		}

		telemetryM.debug("Turn the robot manually to test the Heading PIDF(s).");
		telemetryM.addData("Zero Line", 0);
		telemetryM.addData("Error", follower.errorCalculator.getHeadingError());
		telemetryM.update(telemetry);
	}
}

/**
 * This is the DriveTuner OpMode.
 */
class DriveTuner extends OpMode {
	public static double DISTANCE = 40;
	private boolean forward = true;

	private PathChain forwards;
	private PathChain backwards;

	@Override
	public void init() {
		follower.setStartingPose(new Pose(72, 72));
	}

	@Override
	public void init_loop() {
		telemetryM.debug("This will run the robot in a straight line going " + DISTANCE + "inches forward.");
		telemetryM.debug("The robot will go forward and backward continuously along the path.");
		telemetryM.debug("Make sure you have enough room.");
		telemetryM.update(telemetry);
		follower.update();
		Tuning.drawOnlyCurrent();
	}

	@Override
	public void start() {
		follower.deactivateAllPIDFs();
		follower.activateDrive();

		forwards = follower.pathBuilder()
				.setGlobalDeceleration()
				.addPath(new BezierLine(new Pose(72, 72), new Pose(DISTANCE + 72, 72)))
				.setConstantHeadingInterpolation(0)
				.build();

		backwards = follower.pathBuilder()
				.setGlobalDeceleration()
				.addPath(new BezierLine(new Pose(DISTANCE + 72, 72), new Pose(72, 72)))
				.setConstantHeadingInterpolation(0)
				.build();

		follower.followPath(forwards);
	}

	@Override
	public void loop() {
		follower.update();
		Tuning.draw();

		if (!follower.isBusy()) {
			if (forward) {
				forward = false;
				follower.followPath(backwards);
			} else {
				forward = true;
				follower.followPath(forwards);
			}
		}

		telemetryM.debug("Driving forward?: " + forward);
		telemetryM.addData("Zero Line", 0);
		telemetryM.addData("Error", follower.errorCalculator.getDriveErrors()[1]);
		telemetryM.update(telemetry);
	}
}

/**
 * This is the Line Test Tuner OpMode.
 */
class Line extends OpMode {
	public static double DISTANCE = 40;
	private boolean forward = true;

	private Path forwards;
	private Path backwards;

	@Override
	public void init() {
		follower.setStartingPose(new Pose(72, 72));
	}

	@Override
	public void init_loop() {
		telemetryM.debug("This will activate all the PIDF(s)");
		telemetryM.debug("The robot will go forward and backward continuously along the path while correcting.");
		telemetryM.debug("You can adjust the PIDF values to tune the robot's drive PIDF(s).");
		telemetryM.update(telemetry);
		follower.update();
		Tuning.drawOnlyCurrent();
	}

	@Override
	public void start() {
		follower.activateAllPIDFs();
		forwards = new Path(new BezierLine(new Pose(72, 72), new Pose(DISTANCE + 72, 72)));
		forwards.setConstantHeadingInterpolation(0);
		backwards = new Path(new BezierLine(new Pose(DISTANCE + 72, 72), new Pose(72, 72)));
		backwards.setConstantHeadingInterpolation(0);
		follower.followPath(forwards);
	}

	@Override
	public void loop() {
		follower.update();
		Tuning.draw();

		if (!follower.isBusy()) {
			if (forward) {
				forward = false;
				follower.followPath(backwards);
			} else {
				forward = true;
				follower.followPath(forwards);
			}
		}

		telemetryM.debug("Driving Forward?: " + forward);
		telemetryM.update(telemetry);
	}
}

/**
 * This is the Centripetal Tuner OpMode.
 */
class CentripetalTuner extends OpMode {
	public static double DISTANCE = 20;
	private boolean forward = true;

	private Path forwards;
	private Path backwards;

	@Override
	public void init() {
		follower.setStartingPose(new Pose(72, 72));
	}

	@Override
	public void init_loop() {
		telemetryM.debug("This will run the robot in a curve going " + DISTANCE + " inches to the left and the same number of inches forward.");
		telemetryM.debug("The robot will go continuously along the path.");
		telemetryM.debug("Make sure you have enough room.");
		telemetryM.update(telemetry);
		follower.update();
		Tuning.drawOnlyCurrent();
	}

	@Override
	public void start() {
		follower.activateAllPIDFs();
		forwards = new Path(new BezierCurve(new Pose(72, 72), new Pose(Math.abs(DISTANCE) + 72, 72), new Pose(Math.abs(DISTANCE) + 72, DISTANCE + 72)));
		backwards = new Path(new BezierCurve(new Pose(Math.abs(DISTANCE) + 72, DISTANCE + 72), new Pose(Math.abs(DISTANCE) + 72, 72), new Pose(72, 72)));

		backwards.setTangentHeadingInterpolation();
		backwards.reverseHeadingInterpolation();

		follower.followPath(forwards);
	}

	@Override
	public void loop() {
		follower.update();
		Tuning.draw();
		if (!follower.isBusy()) {
			if (forward) {
				forward = false;
				follower.followPath(backwards);
			} else {
				forward = true;
				follower.followPath(forwards);
			}
		}

		telemetryM.debug("Driving away from the origin along the curve?: " + forward);
		telemetryM.update(telemetry);
	}
}

/**
 * This is the Triangle autonomous OpMode.
 */
class Triangle extends OpMode {
	private final Pose startPose = new Pose(72, 72, Math.toRadians(0));
	private final Pose interPose = new Pose(24 + 72, -24 + 72, Math.toRadians(90));
	private final Pose endPose = new Pose(24 + 72, 24 + 72, Math.toRadians(45));

	private PathChain triangle;

	@Override
	public void loop() {
		follower.update();
		Tuning.draw();

		if (follower.atParametricEnd()) {
			follower.followPath(triangle, true);
		}
	}

	@Override
	public void init() {
		follower.setStartingPose(new Pose(72, 72));
	}

	@Override
	public void init_loop() {
		telemetryM.debug("This will run in a roughly triangular shape, starting on the bottom-middle point.");
		telemetryM.debug("So, make sure you have enough space to the left, front, and right to run the OpMode.");
		telemetryM.update(telemetry);
		follower.update();
		Tuning.drawOnlyCurrent();
	}

	@Override
	public void start() {
		follower.setStartingPose(startPose);

		triangle = follower.pathBuilder()
				.addPath(new BezierLine(startPose, interPose))
				.setLinearHeadingInterpolation(startPose.getHeading(), interPose.getHeading())
				.addPath(new BezierLine(interPose, endPose))
				.setLinearHeadingInterpolation(interPose.getHeading(), endPose.getHeading())
				.addPath(new BezierLine(endPose, startPose))
				.setLinearHeadingInterpolation(endPose.getHeading(), startPose.getHeading())
				.build();

		follower.followPath(triangle);
	}
}

/**
 * This is the Circle autonomous OpMode.
 */
class Circle extends OpMode {
	public static double RADIUS = 10;
	private PathChain circle;

	public void start() {
		circle = follower.pathBuilder()
				.addPath(new BezierCurve(new Pose(72, 72), new Pose(RADIUS + 72, 72), new Pose(RADIUS + 72, RADIUS + 72)))
				.setHeadingInterpolation(HeadingInterpolator.facingPoint(72, RADIUS + 72))
				.addPath(new BezierCurve(new Pose(RADIUS + 72, RADIUS + 72), new Pose(RADIUS + 72, (2 * RADIUS) + 72), new Pose(72, (2 * RADIUS) + 72)))
				.setHeadingInterpolation(HeadingInterpolator.facingPoint(72, RADIUS + 72))
				.addPath(new BezierCurve(new Pose(72, (2 * RADIUS) + 72), new Pose(-RADIUS + 72, (2 * RADIUS) + 72), new Pose(-RADIUS + 72, RADIUS + 72)))
				.setHeadingInterpolation(HeadingInterpolator.facingPoint(72, RADIUS + 72))
				.addPath(new BezierCurve(new Pose(-RADIUS + 72, RADIUS + 72), new Pose(-RADIUS + 72, 72), new Pose(72, 72)))
				.setHeadingInterpolation(HeadingInterpolator.facingPoint(72, RADIUS + 72))
				.build();
		follower.followPath(circle);
	}

	@Override
	public void init_loop() {
		telemetryM.debug("This will run in a roughly circular shape of radius " + RADIUS + ", starting on the right-most edge. ");
		telemetryM.debug("So, make sure you have enough space to the left, front, and back to run the OpMode.");
		telemetryM.debug("It will also continuously face the center of the circle to test your heading and centripetal correction.");
		telemetryM.update(telemetry);
		follower.update();
		Tuning.drawOnlyCurrent();
	}

	@Override
	public void init() {
		follower.setStartingPose(new Pose(72, 72));
	}

	@Override
	public void loop() {
		follower.update();
		Tuning.draw();

		if (follower.atParametricEnd()) {
			follower.followPath(circle);
		}
	}
}

/**
 * This is the Drawing class for Panels Dashboard.
 */
class Drawing {
	public static final double ROBOT_RADIUS = 9;
	private static final FieldManager panelsField = PanelsField.INSTANCE.getField();

	private static final Style robotLook = new Style(
			"", "#3F51B5", 0.75
	);
	private static final Style historyLook = new Style(
			"", "#4CAF50", 0.75
	);

	public static void init() {
		panelsField.setOffsets(PanelsField.INSTANCE.getPresets().getPEDRO_PATHING());
	}

	public static void drawDebug(Follower follower) {
		if (follower.getCurrentPath() != null) {
			drawPath(follower.getCurrentPath(), robotLook);
			Pose closestPoint = follower.getPointFromPath(follower.getCurrentPath().getClosestPointTValue());
			drawRobot(new Pose(closestPoint.getX(), closestPoint.getY(), follower.getCurrentPath().getHeadingGoal(follower.getCurrentPath().getClosestPointTValue())), robotLook);
		}
		drawPoseHistory(follower.getPoseHistory(), historyLook);
		drawRobot(follower.getPose(), historyLook);

		sendPacket();
	}

	public static void drawRobot(Pose pose, Style style) {
		if (pose == null || Double.isNaN(pose.getX()) || Double.isNaN(pose.getY()) || Double.isNaN(pose.getHeading())) {
			return;
		}

		panelsField.setStyle(style);
		panelsField.moveCursor(pose.getX(), pose.getY());
		panelsField.circle(ROBOT_RADIUS);

		Vector v = pose.getHeadingAsUnitVector();
		v.setMagnitude(v.getMagnitude() * ROBOT_RADIUS);
		double x1 = pose.getX() + v.getXComponent() / 2, y1 = pose.getY() + v.getYComponent() / 2;
		double x2 = pose.getX() + v.getXComponent(), y2 = pose.getY() + v.getYComponent();

		panelsField.setStyle(style);
		panelsField.moveCursor(x1, y1);
		panelsField.line(x2, y2);
	}

	public static void drawRobot(Pose pose) {
		drawRobot(pose, robotLook);
	}

	public static void drawPath(Path path, Style style) {
		double[][] points = path.getPanelsDrawingPoints();

		for (int i = 0; i < points[0].length; i++) {
			for (int j = 0; j < points.length; j++) {
				if (Double.isNaN(points[j][i])) {
					points[j][i] = 0;
				}
			}
		}

		panelsField.setStyle(style);
		panelsField.moveCursor(points[0][0], points[0][1]);
		panelsField.line(points[1][0], points[1][1]);
	}

	public static void drawPath(PathChain pathChain, Style style) {
		for (int i = 0; i < pathChain.size(); i++) {
			drawPath(pathChain.getPath(i), style);
		}
	}

	public static void drawPoseHistory(PoseHistory poseTracker, Style style) {
		panelsField.setStyle(style);

		int size = poseTracker.getXPositionsArray().length;
		for (int i = 0; i < size - 1; i++) {
			panelsField.moveCursor(poseTracker.getXPositionsArray()[i], poseTracker.getYPositionsArray()[i]);
			panelsField.line(poseTracker.getXPositionsArray()[i + 1], poseTracker.getYPositionsArray()[i + 1]);
		}
	}

	public static void drawPoseHistory(PoseHistory poseTracker) {
		drawPoseHistory(poseTracker, historyLook);
	}

	public static void sendPacket() {
		panelsField.update();
	}
}