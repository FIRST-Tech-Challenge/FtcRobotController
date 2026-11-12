package org.firstinspires.ftc.teamcode.pedro.procedures;

import com.pedropathing.tuning.autotune.Procedure;
import com.pedropathing.tuning.autotune.TuningOpMode;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver.EncoderDirection;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.PinpointOdometry;

/**
 * Pedro's Pinpoint procedure, ported from Pedro's revhub localizer to V1's PinpointOdometry
 * device lookup and 4-Bar pod resolution. It drives no motors: push or spin the robot by hand.
 * Results are PinpointOdometry constants: setOffsets(x, y) in mm, the same convention as
 * Pedro's PinpointConfig xPodOffset/yPodOffset.
 */
public class PinpointTuner extends Procedure {
    /** Offsets assume a half turn; reject spins far from 180 degrees. */
    static final double MIN_OFFSET_SPIN_DEG = 150;

    public PinpointTuner() {
        super("Pinpoint Tuner", "A procedure for checking the Pinpoint pod directions and offsets.");
    }

    @Override
    public void run() throws InterruptedException {
        Pose2D forward = runOpMode(new PinpointPush("Forward Direction Identification",
                "Determines if your forward pod needs to be reversed. \nPush your robot forward and then stop the Opmode",
                EncoderDirection.FORWARD, EncoderDirection.FORWARD));
        boolean forwardPodReversed = forward.getX(DistanceUnit.INCH) < 0;

        Pose2D left = runOpMode(new PinpointPush("Strafe Direction Identification",
                "Determines if your strafe pod needs to be reversed. \nPush your robot to the left and then stop the Opmode",
                EncoderDirection.FORWARD, EncoderDirection.FORWARD));
        boolean strafePodReversed = left.getY(DistanceUnit.INCH) < 0;

        EncoderDirection xDirection = forwardPodReversed ? EncoderDirection.REVERSED : EncoderDirection.FORWARD;
        EncoderDirection yDirection = strafePodReversed ? EncoderDirection.REVERSED : EncoderDirection.FORWARD;

        Pose2D spun = runOpMode(new PinpointPush("Offsets Identification",
                "Automatically identifies the offsets for your Pinpoint localizer. \nSpin your robot in place 180 degrees counterclockwise and then stop the Opmode",
                xDirection, yDirection));
        double spinDeg = spun.getHeading(AngleUnit.DEGREES);
        if (Math.abs(spinDeg) < MIN_OFFSET_SPIN_DEG)
            abort(String.format("The robot turned %.1f degrees; spin it about 180 degrees counterclockwise in place and try again.", spinDeg));
        // A half turn about the centre moves each zero-offset pod by twice its offset.
        double xOffsetMm = -spun.getY(DistanceUnit.MM) / 2.0;
        double yOffsetMm = -spun.getX(DistanceUnit.MM) / 2.0;

        result("X_POD_DIRECTION", xDirection);
        result("Y_POD_DIRECTION", yDirection);
        result("PINPOINT_X_POD_OFFSET_MM", xOffsetMm);
        result("PINPOINT_Y_POD_OFFSET_MM", yOffsetMm);
        result("Offset spin (deg)", spinDeg);
        result("X_POD_DIRECTION (installed)", PinpointOdometry.X_POD_DIRECTION);
        result("Y_POD_DIRECTION (installed)", PinpointOdometry.Y_POD_DIRECTION);
        result("PINPOINT_X_POD_OFFSET_MM (installed)", PinpointOdometry.PINPOINT_X_POD_OFFSET_MM);
        result("PINPOINT_Y_POD_OFFSET_MM (installed)", PinpointOdometry.PINPOINT_Y_POD_OFFSET_MM);

        code(Language.JAVA, "// PinpointOdometry.java (compare with the measured pod geometry before replacing it)\n" +
                "public static final double PINPOINT_X_POD_OFFSET_MM = " + xOffsetMm + ";\n" +
                "public static final double PINPOINT_Y_POD_OFFSET_MM = " + yOffsetMm + ";\n" +
                "public static final GoBildaPinpointDriver.EncoderDirection X_POD_DIRECTION =\n" +
                "        GoBildaPinpointDriver.EncoderDirection." + xDirection + ";\n" +
                "public static final GoBildaPinpointDriver.EncoderDirection Y_POD_DIRECTION =\n" +
                "        GoBildaPinpointDriver.EncoderDirection." + yDirection + ";");
    }
}

/** Zero pod offsets and the given directions; reports the last pose read before STOP. */
class PinpointPush extends TuningOpMode<Pose2D> {
    private static final Pose2D ZERO = new Pose2D(DistanceUnit.MM, 0, 0, AngleUnit.RADIANS, 0);
    private final EncoderDirection xDirection, yDirection;

    PinpointPush(String name, String description, EncoderDirection xDirection, EncoderDirection yDirection) {
        super(name, description, true);
        this.xDirection = xDirection;
        this.yDirection = yDirection;
    }

    @Override
    protected Pose2D runTuningOpMode() throws InterruptedException {
        // PinpointOdometry supplies V1's device name/aliases and 4-Bar resolution; override the rest.
        GoBildaPinpointDriver pinpoint = new PinpointOdometry(hardwareMap).getDriver();
        pinpoint.setOffsets(0, 0, DistanceUnit.MM);
        pinpoint.setEncoderDirections(xDirection, yDirection);
        pinpoint.resetPosAndIMU();
        Thread.sleep(1000); // Keep still: IMU calibration takes about 0.25 s.
        waitForStart();
        pinpoint.setPosition(ZERO);

        Pose2D pose = ZERO;
        while (!isStopRequested()) {
            pinpoint.update();
            pose = pinpoint.getPosition();
            telemetry.addData("Pose (in, deg)", "%.2f, %.2f | %.1f", pose.getX(DistanceUnit.INCH),
                    pose.getY(DistanceUnit.INCH), pose.getHeading(AngleUnit.DEGREES));
            telemetry.update();
        }
        return pose;
    }
}
