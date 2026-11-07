package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

// Backup auto that talks to the hardware directly so it still works if the main code breaks.
// Each Simple auto extends this and puts its steps in runRoutine().
// Values below can be changed on FTC Dashboard under SimpleAutoBase.
@Config
public abstract class SimpleAutoBase extends LinearOpMode {

    public static double DRIVE_POWER = 0.5;

    // flywheel speed is in ticks/sec
    public static double KP = 120, KI = 0.2, KD = 12, KF = 150;
    public static double LAUNCH_VEL = 1200;
    public static double VEL_TOLERANCE = 50;

    public static double FEED_OPEN = 0.04;
    public static double FEED_CLOSED = 0.3;
    public static double INTAKE_POWER = 0.8;

    // times are in seconds
    public static double READY_TIMEOUT = 2.0;
    public static double FEED_HOLD = 0.3;
    public static double FEED_PAUSE = 0.5;
    public static double SCAN_TIME = 8.0;
    public static double PARK_TIME = 8.0;

    private static final double AUTO_LENGTH = 30;

    private DcMotor fl, bl, fr, br, intake;
    private DcMotorEx launcher;
    private Servo feeder;
    private GoBildaPinpointDriver pinpoint;
    private Limelight3A limelight;

    private final ElapsedTime matchTimer = new ElapsedTime();
    private boolean spinning = false;
    private String lastShot = "none";

    protected abstract void runRoutine();

    @Override
    public void runOpMode() {
        // show telemetry on the Driver Hub and Dashboard
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // drive motors
        fl = hardwareMap.get(DcMotor.class, "FL");
        bl = hardwareMap.get(DcMotor.class, "BL");
        fr = hardwareMap.get(DcMotor.class, "FR");
        br = hardwareMap.get(DcMotor.class, "BR");
        fl.setDirection(DcMotor.Direction.REVERSE);
        bl.setDirection(DcMotor.Direction.REVERSE);
        fr.setDirection(DcMotor.Direction.FORWARD);
        br.setDirection(DcMotor.Direction.FORWARD);
        for (DcMotor m : new DcMotor[]{fl, bl, fr, br}) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        // flywheel, intake and feeder gate
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        launcher.setDirection(DcMotor.Direction.REVERSE);
        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        intake = hardwareMap.get(DcMotor.class, "intake");
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        feeder = hardwareMap.get(Servo.class, "feeder");
        feeder.setPosition(FEED_CLOSED);

        // odometry, keep the robot still while it resets
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.setOffsets(-8.0, 1.0, DistanceUnit.MM);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD);
        pinpoint.resetPosAndIMU();

        // camera for the AprilTag scan
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();
        FtcDashboard.getInstance().startCameraStream(limelight, 10);

        telemetry.addData("Status", "Ready. Keep the robot still.");
        telemetry.update();

        waitForStart();
        matchTimer.reset();

        if (opModeIsActive()) runRoutine();

        // turn everything off at the end
        setDrive(0, 0);
        launcher.setPower(0);
        intake.setPower(0);
        feeder.setPosition(FEED_CLOSED);
    }

    // Drive straight until odometry says we've gone far enough. Negative power drives backward.
    protected void driveInches(double inches, double power) {
        tick();
        double startX = x(), startY = y();
        while (opModeIsActive() && Math.hypot(x() - startX, y() - startY) < inches) {
            setDrive(power, 0);
            tick();
        }
        stopAndPause();
    }

    // Turn until the heading passes the target. 0 is straight ahead, positive is left.
    protected void turnToDegrees(double target, double power) {
        tick();
        boolean left = target > heading();
        while (opModeIsActive() && (left ? heading() < target : heading() > target)) {
            setDrive(0, left ? power : -power);
            tick();
        }
        stopAndPause();
    }

    // Drive for a set time, used to push into the wall.
    protected void driveSeconds(double seconds, double power) {
        ElapsedTime t = new ElapsedTime();
        while (opModeIsActive() && t.seconds() < seconds) {
            setDrive(power, 0);
            tick();
        }
        stopAndPause();
    }

    // Wait but keep odometry, flywheel and telemetry updating.
    protected void waitSeconds(double seconds) {
        ElapsedTime t = new ElapsedTime();
        while (opModeIsActive() && t.seconds() < seconds) tick();
    }

    // Start the flywheel. tick() keeps it at LAUNCH_VEL.
    protected void spinUp() {
        spinning = true;
    }

    // Look for any tag between minId and maxId for up to SCAN_TIME. True means the hive is tipped.
    protected boolean scanForTip(int minId, int maxId) {
        ElapsedTime t = new ElapsedTime();
        while (opModeIsActive() && t.seconds() < SCAN_TIME) {
            if (tagInView(minId, maxId)) return true;
            telemetry.addData("Scan", "%.1fs left", SCAN_TIME - t.seconds());
            tick();
        }
        return false;
    }

    // Shoot count balls. Before each one, wait for the flywheel to get back to speed
    // (or READY_TIMEOUT), then open the gate so the intake pushes a ball in.
    // Stops early if it's time to park.
    protected void shoot(int count) {
        spinning = true;
        intake.setPower(INTAKE_POWER);

        for (int i = 0; i < count && opModeIsActive() && !timeToPark(); i++) {
            ElapsedTime t = new ElapsedTime();
            while (opModeIsActive() && !timeToPark() && !isReady() && t.seconds() < READY_TIMEOUT) {
                tick();
            }
            if (timeToPark()) break;
            lastShot = String.format("Shot %d: %s at %.0f, waited %.1fs", i + 1,
                    isReady() ? "READY" : "TIMEOUT", launcher.getVelocity(), t.seconds());

            feeder.setPosition(FEED_OPEN);
            waitSeconds(FEED_HOLD);
            feeder.setPosition(FEED_CLOSED);
            waitSeconds(FEED_PAUSE);
        }

        spinning = false;
        launcher.setPower(0);
        intake.setPower(0);
    }

    // Runs every loop: update odometry, hold flywheel speed, send telemetry.
    private void tick() {
        pinpoint.update();
        if (spinning) {
            launcher.setVelocityPIDFCoefficients(KP, KI, KD, KF);
            launcher.setVelocity(LAUNCH_VEL);
        }
        telemetry.addData("Time left", "%.1f", AUTO_LENGTH - matchTimer.seconds());
        telemetry.addData("Pose", "x %.1f  y %.1f  h %.1f", x(), y(), heading());
        telemetry.addData("Flywheel", "%.0f / %.0f  ready %b",
                launcher.getVelocity(), spinning ? LAUNCH_VEL : 0, isReady());
        telemetry.addData("Last shot", lastShot);
        telemetry.update();
    }

    // Flywheel is close enough to LAUNCH_VEL to shoot.
    private boolean isReady() {
        return spinning && Math.abs(launcher.getVelocity() - LAUNCH_VEL) < VEL_TOLERANCE;
    }

    // Less than PARK_TIME left in auto, so stop shooting and go park.
    private boolean timeToPark() {
        return AUTO_LENGTH - matchTimer.seconds() <= PARK_TIME;
    }

    // Check whether the Limelight sees a tag in the given ID range right now.
    private boolean tagInView(int minId, int maxId) {
        LLResult r = limelight.getLatestResult();
        if (r == null || !r.isValid()) return false;
        for (LLResultTypes.FiducialResult f : r.getFiducialResults()) {
            int id = f.getFiducialId();
            if (id >= minId && id <= maxId) return true;
        }
        return false;
    }

    // Forward is positive forward, turn is positive left.
    private void setDrive(double forward, double turn) {
        fl.setPower(forward - turn);
        bl.setPower(forward - turn);
        fr.setPower(forward + turn);
        br.setPower(forward + turn);
    }

    private void stopAndPause() {
        setDrive(0, 0);
        tick();
        sleep(250);
    }

    private double x() {
        return pinpoint.getPosition().getX(DistanceUnit.INCH);
    }

    private double y() {
        return pinpoint.getPosition().getY(DistanceUnit.INCH);
    }

    private double heading() {
        return pinpoint.getPosition().getHeading(AngleUnit.DEGREES);
    }
}
