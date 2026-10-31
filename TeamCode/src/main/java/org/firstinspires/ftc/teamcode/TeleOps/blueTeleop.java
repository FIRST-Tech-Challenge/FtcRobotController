package org.firstinspires.ftc.teamcode.TeleOps;


import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Vector;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem_Motor;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystemFF;
import org.firstinspires.ftc.teamcode.subsystems.StopperSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;
import org.firstinspires.ftc.teamcode.utils.OpModeStorage;
import org.firstinspires.ftc.teamcode.utils.TurretTracking;
import org.firstinspires.ftc.teamcode.utils.AutoAim;
import org.firstinspires.ftc.teamcode.utils.AutoAimVision;


@TeleOp(name = "Teleop")
public class blueTeleop extends OpMode {
    private Follower follower;
    private TurretTracking tracking;
    private AutoAim autoAim;
    private AutoAimVision visionAim;


    /**subsystems declaration
     *
     */

    private IntakeSubsystem_Motor intake;
    private ShooterSubsystemFF shooter;
    private TurretSubsystem turret;
    private StopperSubsystem stopper;
    private VisionSubsystem vision;

    /**button edges
     *
     */
    private boolean prevA = false;
    private boolean prevX = false;
    private boolean prevUp = false;
    private boolean prevDown = false;
    private boolean prevPoseButton, prevVisionButton;

    /**Subsystem states
     *
     */
    public boolean intakeOn = true;
    private boolean shooterEnabled = false;
    private static final double DEFAULT_RPM = 3200;
    private static final double RPM_STEP = 250;
    private static final double MAX_RPM = 6000;
    private static final double MIN_RPM = 0;

    private long feedAtNs = 0;

    /**configs
     *
     */
    TurretSubsystem.Config turretConfig = new TurretSubsystem.Config();

    /**
     * Aiming
     */
    // Fill these with the two target positions in your Pedro field coordinates.
    // Goal coordinates in Pedro field coordinates (inches)
    private static final double BLUE_LOWER_GOAL_X = 84.0;
    private static final double BLUE_LOWER_GOAL_Y = 46.0;
    private static final double BLUE_UPPER_GOAL_X = 84.0;
    private static final double BLUE_UPPER_GOAL_Y = 96.0;

    // Fill in your measured red coordinates.
    private static final double RED_LOWER_GOAL_X = Double.NaN;
    private static final double RED_LOWER_GOAL_Y = Double.NaN;
    private static final double RED_UPPER_GOAL_X = Double.NaN;
    private static final double RED_UPPER_GOAL_Y = Double.NaN;

    private double lowerGoalX, lowerGoalY;
    private double upperGoalX, upperGoalY;

    // 0 = shooter points forward along robot +X
    private static final double SHOOTER_OFFSET = 0.0;

    // Assumes the field runs from Y = 0 to Y = 144 inches.
    private static final double FIELD_MIDPOINT_Y = 72.0;

    private static final double DRIVE_DEADBAND = 0.08;

    private enum AimMode {
        NONE,
        POSE,
        VISION
    }

    private AimMode aimMode = AimMode.NONE;

    /**
     * Alliance selection
     */

    private int allianceChoice = 0; // 0 = Auto, 1 = Blue, 2 = Red
    private boolean isBlue = true;


    @Override
    public void init() {
        // Replace with the measured voltage when the turret points forward.
        turretConfig.encoderCenterVolts = 1.5;

        follower = Constants.create(hardwareMap);

        intake = new IntakeSubsystem_Motor(hardwareMap);
        shooter = new ShooterSubsystemFF(hardwareMap);
        stopper = new StopperSubsystem(hardwareMap);
        vision = new VisionSubsystem(hardwareMap);

        visionAim = new AutoAimVision(follower, vision);
        visionAim.setSelectionMode(
                VisionSubsystem.SelectionMode.MOST_TAGS
        );
        //turret = new TurretSubsystem(hardwareMap, turretConfig);

        shooter.setTargetRpm(DEFAULT_RPM);

        /*
        tracking = new TurretTracking(
                turret,
                new TurretTracking.Target(LOWER_TARGET_X, LOWER_TARGET_Y),
                new TurretTracking.Target(UPPER_TARGET_X, UPPER_TARGET_Y),
                FIELD_MIDPOINT_Y
        );

         */
    }

    @Override
    public void init_loop() {
        if (gamepad1.dpad_down && !prevDown) {
            allianceChoice++;
            if (allianceChoice > 2) allianceChoice = 0;
        }

        if (gamepad1.dpad_up && !prevUp) {
            allianceChoice--;
            if (allianceChoice < 0) allianceChoice = 2;
        }

        prevDown = gamepad1.dpad_down;
        prevUp = gamepad1.dpad_up;

        String autoLabel = "Auto (Red)";
        if (OpModeStorage.autonomousIsBlue) {
            autoLabel = "Auto (Blue)";
        }

        String blueLabel = "Blue";
        String redLabel = "Red";

        if (allianceChoice == 0) {
            autoLabel += " <";
        } else if (allianceChoice == 1) {
            blueLabel += " <";
        } else {
            redLabel += " <";
        }

        telemetry.addLine(autoLabel);
        telemetry.addLine(blueLabel);
        telemetry.addLine(redLabel);
        telemetry.update();
    }

    @Override
    public void start() {

        // Select the alliance.
        if (allianceChoice == 0) {
            isBlue = OpModeStorage.autonomousIsBlue;
        } else if (allianceChoice == 1) {
            isBlue = true;
        } else {
            isBlue = false;
        }

        if (isBlue) {
            lowerGoalX = BLUE_LOWER_GOAL_X;
            lowerGoalY = BLUE_LOWER_GOAL_Y;
            upperGoalX = BLUE_UPPER_GOAL_X;
            upperGoalY = BLUE_UPPER_GOAL_Y;
        } else {
            lowerGoalX = RED_LOWER_GOAL_X;
            lowerGoalY = RED_LOWER_GOAL_Y;
            upperGoalX = RED_UPPER_GOAL_X;
            upperGoalY = RED_UPPER_GOAL_Y;
        }

        autoAim = new AutoAim(
                follower,
                lowerGoalX, lowerGoalY,
                upperGoalX, upperGoalY,
                SHOOTER_OFFSET
        );



        // Default pose when starting without Auto.
        Pose startPose = new Pose(72, 72, Math.toRadians(90));

        // Only the Auto option uses the saved pose.
        if (allianceChoice == 0 && OpModeStorage.autonomousEndPose != null) {
            startPose = OpModeStorage.autonomousEndPose;
        }

        follower.setPose(startPose);
        follower.update();

        // Avoid changing RPM if a menu button is still held.
        prevUp = gamepad1.dpad_up;
        prevDown = gamepad1.dpad_down;

        intake.startIntake();
    }

    @Override
    public void loop() {
        /**
         * Drivetrain and pedro
         */

        // All aim transitions occur before the single follower.update().
        double forward = -gamepad1.left_stick_y;
        double strafe  = -gamepad1.left_stick_x;
        double turn    = -gamepad1.right_stick_x;

        if (Math.abs(forward) < DRIVE_DEADBAND) forward = 0;
        if (Math.abs(strafe) < DRIVE_DEADBAND) strafe = 0;
        if (Math.abs(turn) < DRIVE_DEADBAND) turn = 0;

        boolean driverMoving = forward != 0 || strafe != 0 || turn != 0;

        boolean poseButton = gamepad1.y;
        boolean visionButton = gamepad1.right_bumper;
        boolean newPosePress = poseButton && !prevPoseButton;
        boolean newVisionPress = visionButton && !prevVisionButton;
        prevPoseButton = poseButton;
        prevVisionButton = visionButton;


        if (driverMoving) {
            autoAim.cancel();
            visionAim.cancel();
            aimMode = AimMode.NONE;
        } else if (newVisionPress) {
            // Right bumper wins when both buttons are pressed together.
            autoAim.cancel();
            aimMode = AimMode.VISION;
        } else if (newPosePress) {
            visionAim.cancel();
            if (poseGoalCoordinatesSet()) {
                aimMode = AimMode.POSE;
            } else {
                autoAim.cancel();
                aimMode = AimMode.NONE;
            }
        }

        // Only the selected controller receives an active button.
        // Non-selected controllers also receive input updates so their
        // rising-edge detectors reset before switching back.
        boolean poseAiming = autoAim.update(
                aimMode == AimMode.POSE && poseButton,
                driverMoving || aimMode != AimMode.POSE);
        boolean visionAiming = visionAim.update(
                aimMode == AimMode.VISION && visionButton,
                driverMoving || aimMode != AimMode.VISION);

        if (aimMode == AimMode.POSE && !poseAiming) aimMode = AimMode.NONE;
        if (aimMode == AimMode.VISION && !visionAiming) aimMode = AimMode.NONE;

        if (aimMode == AimMode.NONE) {
            follower.manual(new DrivePowers(forward, strafe, turn));
        }

        follower.update();
        Pose robotPose = follower.pose();

        //tracking.update(robotPose.x(), robotPose.y(), robotPose.heading());
        // Math.toDegrees() is a built-in java method

        /*
        need OpModeStorage.autonomousEndPose = follower.pose(); in auto to record pose to transition
         */

        /**
         * Intake Subsystem
         */
        boolean a1 = gamepad1.a;
        if (a1 && !prevA) intakeOn = !intakeOn;
        prevA = a1;

        //main code here moved below shooter

        /**
         * Shooter Subsystem
         */
        // Toggle shooter using X
        boolean x1 = gamepad1.x;

        if (x1 && !prevX) {
            shooterEnabled = !shooterEnabled;
        }

        prevX = x1;
        //RPM adjust
        // Increase target RPM
        boolean up = gamepad1.dpad_up;

        if (up && !prevUp) {
            shooter.setTargetRpm(
                    Math.min(
                            MAX_RPM,
                            shooter.getTargetRpm() + RPM_STEP
                    )
            );
        }

        prevUp = up;

        // Decrease target RPM
        boolean down = gamepad1.dpad_down;

        if (down && !prevDown) {
            shooter.setTargetRpm(
                    Math.max(
                            MIN_RPM,
                            shooter.getTargetRpm() - RPM_STEP
                    )
            );
        }

        prevDown = down;

        shooter.update(shooterEnabled);

        if (gamepad1.leftBumperWasPressed() && shooter.isReady()) {
            intakeOn = false;
            feedAtNs = System.nanoTime() + 300_000_000L;
        }

        if (feedAtNs != 0 && System.nanoTime() >= feedAtNs) {
            intakeOn = true;
            feedAtNs = 0;
        }

        if (intakeOn) intake.startIntake();
        else intake.stopIntake();

        /**
         * Stopper Subsystem
         */
        if (gamepad1.b ) { //&& shooterEnabled && shooter.isReady()
            stopper.on();
        } else {
            stopper.off();
        }

        /**
         * Telemetry
         */
        //robot pose
        telemetry.addData("Robot X", robotPose.x());
        telemetry.addData("Robot Y", robotPose.y());
        telemetry.addData("Heading deg", Math.toDegrees(robotPose.heading()));
        telemetry.addData("Aim mode", aimMode);
        telemetry.addData("Pose aim configured", poseGoalCoordinatesSet());
        telemetry.addData("Vision status", visionAim.getStatus());
        telemetry.addData("Vision correction deg", visionAim.getCommandedCorrectionDegrees());
        telemetry.addData("Vision error deg", visionAim.getHeadingErrorDegrees());
        telemetry.addData("Vision aligned <=2deg", visionAim.isAligned(2.0));

        VisionSubsystem.AimObservation obs = visionAim.getLastObservation();
        if (obs != null) {
            telemetry.addData("Vision CELL", obs.goal);
            telemetry.addData("Tags audience/far", "%d / %d", obs.audienceCount, obs.farCount);
            telemetry.addData("Selected tag IDs", obs.visibleIds);
            telemetry.addData("Tag vec F/L/U (m)", "%.2f / %.2f / %.2f",
                    obs.forwardM, obs.leftM, obs.upM);
        }

        telemetry.addData("Shooter Enabled", shooterEnabled);
        telemetry.addData("Target RPM", shooter.getTargetRpm());
        telemetry.addData("Current RPM", shooter.getVelocityRpm());
        telemetry.addData("Ready", shooter.isReady());

        telemetry.addData("LIVE kP", ShooterSubsystemFF.kP);
        telemetry.addData("LIVE kI", ShooterSubsystemFF.kI);
        telemetry.addData("LIVE kD", ShooterSubsystemFF.kD);
        telemetry.addData("LIVE kV", ShooterSubsystemFF.kV);
        telemetry.addData("LIVE kS", ShooterSubsystemFF.kS);

        telemetry.addData("Motor Power Command", shooter.getPowerCmd());
        telemetry.addData("Shooter Target RPM", shooter.getTargetRpm());
        telemetry.addData("Shooter Actual RPM", shooter.getVelocityRpm());

        /*
        //turret
        telemetry.addData("Target half", tracking.getSelectedHalf());
        telemetry.addData("Target distance (in)", tracking.getDistanceInches());

        telemetry.addData("Requested aim (deg)",
                Math.toDegrees(tracking.getRequestedAngleRadians()));

        telemetry.addData("Measured turret (deg)",
                Math.toDegrees(turret.getAngleRadians()));

        telemetry.addData("Turret state", turret.getState());
        telemetry.addData("Turret aligned", tracking.isAligned());
        */

        telemetry.update();


    }

    private boolean poseGoalCoordinatesSet() {
        return Double.isFinite(lowerGoalX)
                && Double.isFinite(lowerGoalY)
                && Double.isFinite(upperGoalX)
                && Double.isFinite(upperGoalY);
    }

    @Override
    public void stop() {
        if (autoAim != null) autoAim.cancel();
        if (visionAim != null) visionAim.cancel();
        if (vision != null) vision.stop();
    }

}
//lexihere!