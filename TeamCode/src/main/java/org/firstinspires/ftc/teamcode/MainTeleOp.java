package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.methods.Transfer.SHOOTING_VELOCITY;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;

import com.pedropathing.math.Pose;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.teamcode.methods.BatteryVoltage;
import org.firstinspires.ftc.teamcode.methods.BulkReads;
import org.firstinspires.ftc.teamcode.methods.Intake;
import org.firstinspires.ftc.teamcode.methods.Launcher;
import org.firstinspires.ftc.teamcode.methods.OpModeStorage;
import org.firstinspires.ftc.teamcode.methods.Transfer;
import org.firstinspires.ftc.teamcode.methods.ShootingCoordinator;
import org.firstinspires.ftc.teamcode.methods.Turret;
import org.firstinspires.ftc.teamcode.pedro.Constants;

import org.firstinspires.ftc.teamcode.methods.LimelightVision;
import org.firstinspires.ftc.teamcode.methods.CoordinateAim;
import org.firstinspires.ftc.teamcode.methods.CoordinateTurretControl;
import org.firstinspires.ftc.teamcode.methods.ManualTurretAimControl;
import org.firstinspires.ftc.teamcode.methods.TurretHeadingCalibration;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * MainTeleOp
 *
 * ===== GAMEPAD 1 (driver) =====
 *   Sticks      -> field-centric drive
 *   A (hold)    -> aim turret at selected target with GP2 D-pad; release corrects robot heading
 *
 * ===== GAMEPAD 2 (operator) =====
 *   Y              -> toggle launcher flywheels on/off
 *   X (hold)       -> intake
 *   B (hold)       -> reverse intake, transfer and flywheel; release restores normal controls
 *   Right Trigger  -> shoot (hold to keep firing; intake runs while firing)
 *   Right Bumper   -> reverse all three motors (same as B)
 *   Left Trigger   -> intake
 *   Left Bumper    -> hold balls
 *   D-pad          -> turret control while TurretAuto is off
 *                       left/right = rotate, up/down = pitch
 *          thx claude <3
 */
@TeleOp(name = "MainTeleOp")
public class MainTeleOp extends OpMode {

    // How far a trigger must be pressed to count as "pressed"
    private static final double TRIGGER_THRESHOLD = 0.2;

    //Methods
    private Follower follower;
    private Launcher launcher;

    private String[] vals = {"Amplitude", "y-int"};

    private String currTuning = vals[0];

    private double amplitude = 170;
    private int yint = 1375;

    private Intake intake;
    private Turret turret;
    private DcMotorEx turretMotor;
    private final ManualTurretAimControl manualAim = new ManualTurretAimControl();
    private String headingCalibrationStatus = "Not requested";
    private double headingCorrectionDeg = 0;
    private boolean coordinateAimEnabled;
    private boolean manualTurret;
    private LimelightVision vision;
    private Servo led;

    private Transfer transfer;
    private ShootingCoordinator shooting;
    private BulkReads bulk;

    private String yawSource = "Pedro";
    private IMU imu;
    private boolean imuAvailable = false;

    //turret stuff
    private double kP = 0.01;
    private double kD = 0.0;

    private final ElapsedTime yawTimer = new ElapsedTime();
    private boolean haveLastHeading = false;

    double[] target = {57.4, 82.3};

    //teleop swithcing sides variables
    private static final double RED_FORWARD_DEG  = 180;

    private static final double BLUE_FORWARD_DEG = 180;

    private double driverOffsetRad = 0;
    private boolean lastOptions1 = false;

    private double lastHeadingRad = 0;
    private double robotYawRateDegPerSec = Double.NaN;

    private boolean isRumbling = false;

    public String allegiance;

    private boolean wasOn = false;

    boolean isManual;

    private void readYaw(double headingRad) {
        double dt = yawTimer.seconds();
        yawTimer.reset();
        robotYawRateDegPerSec = Double.NaN;
        if (haveLastHeading && Double.isFinite(headingRad)
                && dt >= 0.005 && dt <= 0.25) {
            double delta = Math.atan2(
                    Math.sin(headingRad - lastHeadingRad),
                    Math.cos(headingRad - lastHeadingRad));
            robotYawRateDegPerSec = Math.toDegrees(delta / dt);
        }
        lastHeadingRad = headingRad;
        haveLastHeading = Double.isFinite(headingRad);
    }

    @Override
    public void init() {
        //hardware mapping
        follower = Constants.create(hardwareMap);
        led = hardwareMap.get(Servo.class, "led");
        // One shared, rate-limited battery reading instead of per-class reads every loop.
        BatteryVoltage battery = new BatteryVoltage(hardwareMap);
        launcher    = new Launcher(hardwareMap, battery);
        transfer = new Transfer(hardwareMap, battery);
        intake      = new Intake(hardwareMap, battery);
        shooting = new ShootingCoordinator(launcher, transfer, intake);
        turret      = new Turret(hardwareMap);
        turretMotor = hardwareMap.get(DcMotorEx.class, "TurretRot");
        coordinateAimEnabled = OpModeStorage.turretForwardZeroConfirmed;
        // Auto handoff supplies the forward-zero count, so begin tracking immediately.
        // Manual target aiming corrects heading only; it requires this existing forward zero.
        manualTurret = false;
        manualAim.reset();
        headingCalibrationStatus = "Not requested";
        headingCorrectionDeg = 0;

        //setting up follower from autos
        follower.setPose(OpModeStorage.autonomousEndPose);   // start where Auto ended
        allegiance = OpModeStorage.LastAllegiance;
        if (allegiance.equals("BLUE")) {
            driverOffsetRad = Math.toRadians(BLUE_FORWARD_DEG);
        } else if(allegiance.equals("RED")) {
            driverOffsetRad = Math.toRadians(RED_FORWARD_DEG);
        } else {
            driverOffsetRad = Math.toRadians(RED_FORWARD_DEG);
        }

        int pipeline = "BLUE".equals(allegiance) ? 2 : 1;

        vision = new LimelightVision(hardwareMap, 4);

        telemetry.setMsTransmissionInterval(100);

        //setting up Lime Light
        try {
            imu = hardwareMap.get(IMU.class, "imu");
            imuAvailable = true;
        } catch (RuntimeException ex) {
            imuAvailable = false; // Pedro heading-rate fallback
        }

        // MANUAL bulk caching: one hub read per loop for every encoder (cleared at loop start).
        bulk = new BulkReads(hardwareMap);

        //setting up turret auto stuff
        //Turret.ROT_MAX_POWER = 0.2;
        //Turret.ROBOT_YAW_FF_GAIN = 0.003;
    }

    @Override
    public void start() {
        follower.manual();
        follower.update();
        vision.start();
        turret.mainPD.reset();
        haveLastHeading = false;
        yawTimer.reset();
    }

    @Override
    public void loop() {
        bulk.clear();   // FIRST: fresh encoder data for this loop

        turret.update();

        //drive code
        if (gamepad1.options && !lastOptions1) {
            driverOffsetRad = follower.pose().heading();
        }
        lastOptions1 = gamepad1.options;

        DrivePowers powers = ManualDrive.fieldCentric(
                gamepad1.left_stick_y,
                gamepad1.left_stick_x,
                -gamepad1.right_stick_x,
                follower.pose().heading() - driverOffsetRad
        );
        follower.manual(powers);
        ManualDrive.driveOrHold(follower, powers);
        follower.update();
        // Optical target motion caused by robot translation (Pedro pose is inches).
        // Gain defaults to ZERO, so existing competition behavior is unchanged.
        turret.mainPD.updateRobotTranslationFromPose(
                follower.pose().x(), follower.pose().y(), follower.pose().heading());
        readYaw(follower.pose().heading());

        vision.update();

        // Consume Y's edge during reverse, without changing the prior on/off state.
        boolean toggleLauncher = gamepad2.yWasPressed();
        boolean outtakeRequested = gamepad2.b || gamepad2.right_bumper;
        if (!outtakeRequested && toggleLauncher) launcher.toggleOn();
        launcher.updateDistanceTarget(follower.pose().x(), follower.pose().y());
        shooting.update(gamepad2.right_trigger > TRIGGER_THRESHOLD,
                gamepad2.left_trigger > 0.1, outtakeRequested, false);
        telemetry.addData("Shooting feed state", shooting.state());
        

// Intake follows the launcher's updated firing/ejecting state.
// Stop it if the launcher reports a fault.
        if (gamepad2.x || launcher.isFiring()) {
            //intake.intake();
        } /*else if (gamepad2.b || launcher.isEjecting()) {
            intake.outtake();
        } else {
            intake.stop();
        }*/

        //ramp
        if (gamepad2.aWasPressed()) {
            intake.toggleRamp();
        }

        if(gamepad2.xWasPressed()) {
            intake.holdBalls();
        }

        ManualTurretAimControl.Result turretOverride = manualAim.update(
                gamepad1.a, System.nanoTime() / 1e9,
                turretMotor.getVelocity() / CoordinateTurretControl.TICKS_PER_DEGREE,
                turretMotor.getCurrentPosition());
        manualTurret = turretOverride.manual || turretOverride.settling;
        if (turretOverride.enteredManual || turretOverride.calibrateHeading) {
            turret.stopRotation();
            turret.mainPD.reset();
        }

        if (gamepad1.bWasPressed()) {
            launcher.stopFire();
        }

        if (gamepad1.xWasPressed()) {
            if (target[0] == 57.4) {
                target[0] = 58;
                target[1] = 56.7;
            }
            else {
                target[0] = 57.4;
                target[1] = 82.3;
            }
        }

        if (turretOverride.calibrateHeading) {
            Pose before = follower.pose();
            double aimedAngleDeg = (turretOverride.encoderTicks
                    - (double) OpModeStorage.turretForwardZeroTicks)
                    / CoordinateTurretControl.TICKS_PER_DEGREE;
            double correctedHeading = TurretHeadingCalibration.headingRad(
                    before.x(), before.y(), target[0], target[1], aimedAngleDeg);
            if (OpModeStorage.turretForwardZeroConfirmed && Double.isFinite(correctedHeading)) {
                headingCorrectionDeg = Math.toDegrees(Math.atan2(
                        Math.sin(correctedHeading - before.heading()),
                        Math.cos(correctedHeading - before.heading())));
                follower.setPose(new Pose(before.x(), before.y(), correctedHeading));
                // Rebase an active chassis hold so it cannot steer toward the old heading.
                if (follower.holding()) follower.hold(follower.pose());
                haveLastHeading = false;
                robotYawRateDegPerSec = Double.NaN;
                yawTimer.reset();
                turret.mainPD.reset();
                headingCalibrationStatus = "Applied";
            } else {
                headingCalibrationStatus = OpModeStorage.turretForwardZeroConfirmed
                        ? "Skipped: invalid target geometry or turret angle"
                        : "Skipped: forward zero must come from Auto";
            }
        }

        // Coordinate target baseline; Limelight refinement can be added after validation.
        CoordinateAim.Result coordinateAim = CoordinateAim.calculate(
                follower.pose().x(), follower.pose().y(), follower.pose().heading(),
                target[0], target[1]);
        double turretAngleDeg = (turretMotor.getCurrentPosition()
                - (double) OpModeStorage.turretForwardZeroTicks)
                / CoordinateTurretControl.TICKS_PER_DEGREE;
        CoordinateTurretControl.Result coordinateControl = CoordinateTurretControl.calculate(
                coordinateAim.relativeBearingDeg, turretAngleDeg, 1, 0.04);
        double coordinatePower = CoordinateTurretControl.motorPower(
                turretAngleDeg, coordinateControl.power, -1);
        if (turretOverride.manual) {
            turret.manualControl(gamepad2.dpad_left, gamepad2.dpad_right);
        } else if (turretOverride.settling) {
            turret.stopRotation();
        } else if (coordinateAimEnabled && coordinateAim.valid && coordinateControl.reachable) {
            turret.setRotationPower(coordinatePower);
        } else {
            turret.stopRotation();
        }
        telemetry.addData("Turret forward zero confirmed", OpModeStorage.turretForwardZeroConfirmed);
        telemetry.addData("Turret control mode", turretOverride.manual ? "MANUAL: G1 A held"
                : turretOverride.settling ? "SETTLING FOR HEADING CORRECTION"
                : coordinateAimEnabled ? "COORDINATE AUTO" : "FORWARD ZERO REQUIRED FROM AUTO");
        telemetry.addData("Heading calibration", headingCalibrationStatus);
        telemetry.addData("Last heading correction (deg)", headingCorrectionDeg);
        telemetry.addData("Selected target X/Y", "%.1f / %.1f", target[0], target[1]);
        telemetry.addData("Turret zero ticks", OpModeStorage.turretForwardZeroTicks);
        telemetry.addData("Coordinate relative bearing", coordinateAim.relativeBearingDeg);
        telemetry.addData("Coordinate turret error", coordinateControl.errorDeg);

        telemetry.addData("Yaw Rate", robotYawRateDegPerSec);

        telemetry.addData("Current", currTuning);
        telemetry.addData("Amplitude", amplitude);
        telemetry.addData("Y-Int", yint);

        /*
        //adjust ramp
        intake.adjustRamp(
                gamepad2.dpad_up,
                gamepad2.dpad_down);
*/




        //turret auto sight loss correction
        //updateRobotYawRate(follower.pose().heading());
        /*yawSource = "Pedro fallback";
        if (imuAvailable) {
            try {
                AngularVelocity av = imu.getRobotAngularVelocity(AngleUnit.DEGREES);
                if (av != null && Double.isFinite(av.zRotationRate)) {
                    robotYawRateDegPerSec = av.zRotationRate;
                    yawSource = "Hub IMU";
                }
            } catch (RuntimeException ex) {
                yawSource = "Pedro fallback";
            }
        }*/

        /*//led updates
        if(launcher.isFlywheelsOn() && !manualTurret && coordinateAimEnabled *//*&& vision.hasAimingTarget()*//*)
            led.setPosition(0.611); //blue
        else if(launcher.isFlywheelsOn() && (manualTurret || !coordinateAimEnabled))
            led.setPosition(0.333); //orange
        else if (!launcher.isFlywheelsOn() && !manualTurret && coordinateAimEnabled)
            led.setPosition(1); //white?
        else
            led.setPosition(0.500); //green*/

        if(target[1] == 56.7 && allegiance.equals("BLUE")) //blue
            led.setPosition(0.60); //Blue
        else if(target[0] == 57.4 && allegiance.equals("BLUE"))
            led.setPosition(0.50); //Cyan
        else if(target[1] == 56.7 && allegiance.equals("RED"))
            led.setPosition(.70); //Red
        else if (target[0] == 57.4 && allegiance.equals("RED"))
            led.setPosition(0.50); //Magenta


        //rumble update
        if(launcher.flywheelAtSpeed()) {
            gamepad2.rumble(1.0, 1.0, Gamepad.RUMBLE_DURATION_CONTINUOUS);
            isRumbling = true;
        } else if(launcher.ballWasShot()) {
            gamepad2.rumble(0.5, 0.5, Gamepad.RUMBLE_DURATION_CONTINUOUS);
            isRumbling = true;
        } else {
            if(isRumbling) {
                gamepad2.stopRumble();
                isRumbling = false;
            }
        }

        //telemetry updates
        telemetry.addData("Allegiance: ", allegiance);
        telemetry.addLine("==========================");
        telemetry.addData("X", follower.pose().x());
        telemetry.addData("Y", follower.pose().y());
        telemetry.addData("Heading", Math.toDegrees(follower.pose().heading()));
        telemetry.addData("Launcher On", launcher.isFlywheelsOn());
        telemetry.addData("Outtake", outtakeRequested);
        telemetry.addData("Firing", launcher.isFiring());
        telemetry.addData("Ramp Down", !intake.isRampUp());
        telemetry.addData("TurretAuto", manualTurret ? "Manual" : (coordinateAimEnabled ? "Coordinate aim" : "Zero required"));
        telemetry.addData("Ramp Pos", intake.rampthingy());
        telemetry.addData("Turret Rot", turret.getRotationDeg());
        telemetry.addData("r trig thresh", gamepad2.right_trigger);
        telemetry.addLine("I <3 The Skin");
        telemetry.addData("Current Velo", launcher.getFlywheelVelo());
        telemetry.addData("Target Velo", launcher.getTargetVelocity());
        telemetry.addData("Error", launcher.getError());
        telemetry.addData("Transfer Velo", transfer.getFlywheelVelo());
        telemetry.addData("Target Transfer Velo", transfer.getTargetVelocity());
        telemetry.addData("Is Targeting?",  vision.hasAimingTarget());
        telemetry.update();
    }

    @Override
    public void stop() {
        OpModeStorage.autonomousEndPose = follower.pose();
        launcher.stopAll();
        transfer.stopAll();
        intake.stop();
        turret.stop();
        vision.stop();
    }
}
