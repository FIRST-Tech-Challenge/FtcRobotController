package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

///Full Autonomous Path Follower OpMode
@Autonomous(name = "LQR Path Follow")
public class PathFollowerOpMode extends LinearOpMode {

// Measured top physical capabilities of your chassis
    // 25 pound, 435 rpm, 144 wheel diameter
   // private static final double MAX_VEL = 93.25;   //in/sec        (depends on bot)
    //private static final double MAX_ACCEL = 489.65; //in/sec^2     (depends on bot)
    //private static final double MAX_ANGULAR = 7.4; //radians/sec   (depends on bot)

    // 25 pound, 312 rpm, 144 wheel diameter
    private static final double MAX_VEL = 46;   //in/sec        (depends on bot)
    private static final double MAX_ACCEL = 40; //in/sec^2     (depends on bot)
    private static final double MAX_ANGULAR = 3.3; //radians/sec   (depends on bot)
// Target arrival thresholds (Tolerances for completion check)
    private static final double DIST_TOL = 1.0;   //finish if within 1 inch of target
    private static final double ANG_TOL = 0.06;   //finish if within 0.06 radians (~3 deg) of target

    private DcMotor fL, fR, bL, bR;
    private VoltageSensor batterySensor;

    @Override
    public void runOpMode() throws InterruptedException {
        fL  = hardwareMap.get(DcMotor.class, "fL");
        fR = hardwareMap.get(DcMotor.class, "fR");
        bL   = hardwareMap.get(DcMotor.class, "bL");
        bR  = hardwareMap.get(DcMotor.class, "bR");

        batterySensor = hardwareMap.voltageSensor.iterator().next();

///check
        // Same as DriverAssistOpMode. With all four FORWARD, a "go forward" command makes the left wheels spin the wrong way and the robot just rotates.
        fL.setDirection(DcMotor.Direction.REVERSE);
        bL.setDirection(DcMotor.Direction.REVERSE);

        // Without this the motors coast when power hits zero instead of resisting, which is the "spins a bit before stopping".
        fL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

// Build our position mapping trackers
        Localizer localizer = createLocalizer();

// Design the total autonomous route path by chaining splines and straight lines together
        PathChain route = new PathChain(
                QuinticHermitePath.fromTans(
                        new Vec2(-56, -56), new Vec2(90, 0), // Start position and path exit direction vector
                        new Vec2(0, -24), new Vec2(90, 0)), // Spline midpoint destination and entry direction vector
                new LinePath(new Vec2(0, -24), new Vec2(48, -24)) // Connect a straight line to the end zone
        );

// Run structural seam validation scans across joints before starting the match
        for (PathChain.Joint j : route.validate()) {
            telemetry.addLine(j.toString());
        }

// Initialize our core LQR movement engine using our measured limits
        LQRPathFollower follower = LQRPathFollower.withDefaults(MAX_VEL, MAX_ACCEL, MAX_ANGULAR);

// Optional step configurations: Uncomment once your team measures exact parameters
        // follower.setChassisDynamics(ChassisDynamics.estimatedDefaults());
        // follower.setLatencyCompensation(0.04);

// Print initial route distance properties to the screen dashboard
        telemetry.addData("Route", "%s, %.1f in, smooth=%s", route.name(), route.totalLength(), route.isSmooth());
        telemetry.update();

// Pause and wait for the driver to tap the active 'Start' button on the station phone
        waitForStart();
        if (isStopRequested()) {
            return;
        }

// Snap a baseline coordinate read and push our path shape goals directly to the tracker memory
        // Tell odometry where the robot really starts. Without this it reads (0, 0) while the path starts at (-56, -56),
        // so the first thing the follower does is try to drive about 80 inches to "catch up" to the path.
        Vec2 startPt = route.pos(0.0);
        localizer.setPose(new Pose(startPt.x, startPt.y, 0));
        localizer.update();
        follower.followPath(route, 0, Math.PI / 2); // Follow path, start facing 0 rad, end facing 90 deg (PI/2)

// Setup our loop clock timer tracker
        ElapsedTime loopTimer = new ElapsedTime();
        loopTimer.reset();

        while (opModeIsActive()) {
//Measure the exact elapsed loop slice duration interval 'dt' in seconds
            double dt = loopTimer.seconds();
            loopTimer.reset();
            if (dt <= 0) {
                dt = 0.02; // Safe fallback framework value if timer registers zero
            }

//Pull fresh hardware positioning inputs and live voltage metrics
            localizer.update();
            Pose pose = localizer.getPose();
            double volts = batterySensor.getVoltage();

//Run LQR update computations to generate required wheel power distribution maps
            double[] powers = follower.update(pose.x, pose.y, pose.h, dt, volts);

//Check the calculated values for invalid NaN (Not a Number) errors
            boolean valid = true;
            for (double p : powers) {
                if (Double.isNaN(p) || Double.isInfinite(p)) {
                    valid = false;
                }
            }
            if (!valid) {
                setMotorPowers(0, 0, 0, 0); // Safe lock: immediately stop motors to prevent runaways
                telemetry.addLine("WARNING: invalid power computed, forcing zero");
                telemetry.update();
                continue; // Jump directly to the next loop iteration cycle
            }

// Assign verified power configurations straight out to the physical wheels
            setMotorPowers(powers[0], powers[1], powers[2], powers[3]);

// Calculate current distance tracking deviations from our targeted path profile references
            LQRPathFollower.Reference ref = follower.referenceAt(follower.elapsed());
            double distErr = Math.hypot(ref.p.x - pose.x, ref.p.y - pose.y);
            double angErr = Math.abs(LQRPathFollower.normalizeAngle(ref.h - pose.h));

//Must finish path timeline AND settle within bounds
            if (follower.isFinished() && distErr < DIST_TOL && angErr < ANG_TOL) {
                break; // Break the execution loop, completing the autonomous path successfully
            }

            telemetry.addData("Segment", "%d / %d", follower.currentSegment() + 1, route.segmentCount());
            telemetry.addData("Progress", "%.2f / %.2f s", follower.elapsed(), follower.duration());
            telemetry.addData("Pose", pose.toString());
            telemetry.addData("Error", "%.2f in / %.3f rad", distErr, angErr);
            telemetry.addData("Battery", "%.2f V", volts);
            telemetry.addData("Loop Speed", "%.0f Hz", 1.0 / dt);

            if (follower.isSaturated()) {
                telemetry.addLine("saturated, integral frozen");
            }
            telemetry.update();
        }

        setMotorPowers(0, 0, 0, 0);
    }
    private Localizer createLocalizer() {
        return new LocalizerAdapters.PinpointLocalizer(hardwareMap, "pinpoint");
    }

    // Unified helper interface that updates all 4 motor power ports simultaneously
    private void setMotorPowers(double frl, double frr, double bal, double br) {
        fL.setPower(frl);
        fR.setPower(frr);
        bL.setPower(bal);
        bR.setPower(br);
    }
}