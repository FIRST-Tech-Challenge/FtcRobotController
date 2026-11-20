package org.firstinspires.ftc.teamcode.pedro.procedures;

import com.pedropathing.math.Pose;
import com.pedropathing.revhub.localizers.OctoQuadConfig;
import com.pedropathing.revhub.localizers.OctoQuadLocalizer;
import com.pedropathing.tuning.autotune.Inputs;
import com.pedropathing.tuning.autotune.Procedure;
import com.pedropathing.tuning.autotune.TuningOpMode;
import com.pedropathing.utils.Angle;
import com.qualcomm.hardware.digitalchickenlabs.OctoQuad;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.List;

public class OctoQuadTuner extends Procedure {
    enum PodType {
        SWING_ARM,
        FOUR_BAR,
        CUSTOM
    }

    public static double SWING_ARM = 336.877962768;
    public static double FOUR_BAR = 505.316944406;

    public OctoQuadTuner() {
        super("OctoQuad Tuner", "A procedure for tuning the OctoQuad localizer.");
    }

    @Override
    public void run() throws InterruptedException {
        Inputs inputs = inputs("Setup", "Set OctoQuad HardwareMap Name and Odometry Pod Type");
        Inputs.Field<String> octoquadName = inputs.s("HardwareMap Name").withDefault("octoquad");
        Inputs.Field<Integer> xPort =  inputs.i("Forward Pod Port").withDefault(0);
        Inputs.Field<Integer> yPort =  inputs.i("Strafe Pod Port").withDefault(1);
        Inputs.Field<OctoQuadTuner.PodType> podType = inputs.e("Odometry Pod Type", OctoQuadTuner.PodType.class).withDefault(OctoQuadTuner.PodType.FOUR_BAR);
        Inputs.Field<OctoQuad.I2cRecoveryMode> recoveryMode = inputs.e("Recovery Mode", OctoQuad.I2cRecoveryMode.class).withDefault(OctoQuad.I2cRecoveryMode.MODE_1_PERIPH_RST_ON_FRAME_ERR);
        awaitInputs(inputs);

        double customPodScalar = 0;

        Inputs inputsHeadingScalar = inputs("Custom Scalar Identification Turns", "Set the number of times you will turn your robot.");
        Inputs.Field<Integer> turns = inputsHeadingScalar.i("Turns").withDefault(10);
        awaitInputs(inputsHeadingScalar);
        double headingScalar = runOpMode(new OctoQuadHeadingScalar(octoquadName.get(), turns.get(), xPort.get(), yPort.get()));

        if (podType.get() == PodType.CUSTOM) {
            Inputs inputsCustom = inputs("Custom Scalar Identification Push Distance", "Set the distance you will push your robot forward in inches");
            Inputs.Field<Double> distance = inputsCustom.d("Distance").withDefault(48.0);
            awaitInputs(inputsCustom);
            customPodScalar = runOpMode(new OctoQuadCustomPodScalar(distance.get(), octoquadName.get(), xPort.get(), yPort.get()));
        }

        boolean forwardPodReversed = runOpMode(new OctoQuadForwardDirection(octoquadName.get(), podType.get(), customPodScalar, headingScalar, xPort.get(), yPort.get()));
        boolean strafePodReversed = runOpMode(new OctoQuadStrafeDirection(octoquadName.get(), podType.get(), customPodScalar, headingScalar, xPort.get(), yPort.get()));

        List<Double> offsets = runOpMode(new OctoQuadOffsets(octoquadName.get(), podType.get(), customPodScalar, forwardPodReversed, strafePodReversed, headingScalar, xPort.get(), yPort.get()));

        result("name", octoquadName.get());
        result("headingScalar", headingScalar);

        if (podType.get() == PodType.CUSTOM) {
            result("podType", "Custom");
            result("ticksPerUnit", customPodScalar);
        } else {
            result("podType", podType.get() == PodType.SWING_ARM ? SWING_ARM : FOUR_BAR);
        }

        result("xPodDirection", forwardPodReversed ? OctoQuad.EncoderDirection.REVERSE : OctoQuad.EncoderDirection.FORWARD);
        result("yPodDirection", strafePodReversed ? OctoQuad.EncoderDirection.REVERSE : OctoQuad.EncoderDirection.FORWARD);
        result("xPodOffset", offsets.get(0));
        result("yPodOffset", offsets.get(1));

        code(Language.JAVA,"public static OctoQuadConfig localizerConfig = new OctoQuadConfig(c -> {\n" +
                "    c.name.set(\"" + octoquadName.get() + "\");\n" +
                "    c.xPodPort.set(" + xPort.get() + ");\n" +
                "    c.yPodPort.set(" + yPort.get() + ");\n" +
                (podType.get() == PodType.CUSTOM ? "    c.ticksPerUnit.set(" + customPodScalar + ");\n" : "    c.ticksPerUnit.set(" + (podType.get() == OctoQuadTuner.PodType.SWING_ARM ? SWING_ARM : FOUR_BAR) + ");\n") +
                "    c.xPodOffset.set(" + offsets.get(0) + ");\n" +
                "    c.yPodOffset.set(" + offsets.get(1) + ");\n" +
                "    c.xPodDirection.set(" + (forwardPodReversed ? "OctoQuad.EncoderDirection.REVERSE" : "OctoQuad.EncoderDirection.FORWARD") + ");\n" +
                "    c.yPodDirection.set(" + (strafePodReversed ? "OctoQuad.EncoderDirection.REVERSE" : "OctoQuad.EncoderDirection.FORWARD") + ");\n" +
                "    c.globalDistanceUnit.set(DistanceUnit.INCH);\n" +
                "    c.offsetUnits.set(DistanceUnit.INCH);\n" +
                "    c.i2cRecoveryMode.set(OctoQuad.I2cRecoveryMode." + recoveryMode.get() + ");\n" +
                "    c.headingScalar.set(" + headingScalar + ");\n" +
                "});");
    }
}

class OctoQuadHeadingScalar extends TuningOpMode<Double> {
    String name;
    int turns;
    double totalHeading = 0;
    double prevHeading = 0;
    int xPodPort;
    int yPodPort;

    public OctoQuadHeadingScalar(String name, int turns, int xPodPort, int yPodPort) {
        super("Heading Scalar Identification",
                "Determines the scalar for the custom pods of the OctoQuad localizer. \n"
                        + "Turn your robot " + turns * 360 + " degrees exactly ("+ turns + " times) exactly and then stop the OpMode.",
                true);
        this.name = name;
        this.turns = turns;
        this.xPodPort = xPodPort;
        this.yPodPort = yPodPort;
    }

    @Override
    protected Double runTuningOpMode() throws InterruptedException {
        OctoQuadConfig config = new OctoQuadConfig(c -> {
            c.name.set(name);
            c.xPodPort.set(xPodPort);
            c.yPodPort.set(yPodPort);
            c.ticksPerUnit.set(1.0);
            c.xPodDirection.set(OctoQuad.EncoderDirection.FORWARD);
            c.yPodDirection.set(OctoQuad.EncoderDirection.FORWARD);
            c.xPodOffset.set(0.0);
            c.yPodOffset.set(0.0);
        });

        OctoQuadLocalizer localizer = new OctoQuadLocalizer(hardwareMap, config);
        localizer.setPose(new Pose(0, 0));
        localizer.update();
        waitForStart();
        while (!isStopRequested()) {
            localizer.update();

            if (localizer.pose().x() != Pose.zero().x() || localizer.pose().y() != Pose.zero().y() || localizer.pose().heading() != Pose.zero().heading()) {
                double currentHeading = localizer.pose().heading();
                totalHeading += Angle.normalizeSigned(currentHeading - prevHeading);
                prevHeading = currentHeading;
            }
        }
        return Math.abs((turns * Math.PI * 2 / totalHeading));
    }
}

class OctoQuadCustomPodScalar extends TuningOpMode<Double> {
    String name;
    double distance;
    int xPodPort, yPodPort;

    public OctoQuadCustomPodScalar(Double distance, String name, int xPodPort, int yPodPort) {
        super("Custom Scalar Identification",
                "Determines the scalar for the custom pods of the OctoQuad localizer. \n"
                        + "Push your robot forward " + distance + " inches exactly and then stop the Opmode",
                true);
        this.name = name;
        this.distance = distance;
        this.xPodPort = xPodPort;
        this.yPodPort = yPodPort;
    }

    @Override
    protected Double runTuningOpMode() {
        OctoQuadConfig config = new OctoQuadConfig(c -> {
            c.name.set(name);
            c.xPodPort.set(xPodPort);
            c.yPodPort.set(yPodPort);
            c.ticksPerUnit.set(1.0);
            c.xPodDirection.set(OctoQuad.EncoderDirection.FORWARD);
            c.yPodDirection.set(OctoQuad.EncoderDirection.FORWARD);
            c.xPodOffset.set(0.0);
            c.yPodOffset.set(0.0);
        });
        OctoQuadLocalizer localizer = new OctoQuadLocalizer(hardwareMap, config);
        localizer.setPose(new Pose(0, 0));
        waitForStart();

        double startTicks = localizer.octoQuad.readAllEncoderData().positions[xPodPort];
        double lastLastTicksPerInch = 0;
        double lastTicksPerInch = 0;

        while (!isStopRequested()) {
            localizer.update();

            double pos = localizer.octoQuad.readAllEncoderData().positions[xPodPort];
            lastLastTicksPerInch = lastTicksPerInch;
            lastTicksPerInch = Math.abs(pos - startTicks) / distance;
        }

        return lastLastTicksPerInch;
    }
}

class OctoQuadForwardDirection extends TuningOpMode<Boolean> {
    String name;
    OctoQuadTuner.PodType podType;
    double customPodScalar;
    double headingScalar;
    int xPodPort, yPodPort;

    public OctoQuadForwardDirection(String name, OctoQuadTuner.PodType podType, double customPodScalar, double headingScalar,
                                    int xPodPort, int yPodPort) {
        super("Forward Direction Identification",
                "Determines if your forward pod needs to be reversed. \n"
                        + "Push your robot forward and then stop the Opmode",
                true);
        this.name = name;
        this.podType = podType;
        this.customPodScalar = customPodScalar;
        this.headingScalar = headingScalar;
        this.xPodPort = xPodPort;
        this.yPodPort = yPodPort;
    }

    @Override
    protected Boolean runTuningOpMode() {
        OctoQuadConfig config = new OctoQuadConfig(c -> {
            c.name.set(name);
            c.xPodPort.set(xPodPort);
            c.yPodPort.set(yPodPort);
            c.xPodDirection.set(OctoQuad.EncoderDirection.FORWARD);
            c.yPodDirection.set(OctoQuad.EncoderDirection.FORWARD);
            c.xPodOffset.set(0.0);
            c.yPodOffset.set(0.0);
            c.headingScalar.set(headingScalar);
            if (podType == OctoQuadTuner.PodType.CUSTOM) {
                c.ticksPerUnit.set(customPodScalar);
            } else {
                c.ticksPerUnit.set(podType == OctoQuadTuner.PodType.SWING_ARM ? OctoQuadTuner.SWING_ARM : OctoQuadTuner.FOUR_BAR);
            }
        });
        OctoQuadLocalizer localizer = new OctoQuadLocalizer(hardwareMap, config);
        localizer.setPose(new Pose(0, 0));
        waitForStart();
        while (!isStopRequested()) {
            localizer.update();
        }

        return localizer.pose().x() < 0;
    }
}

class OctoQuadStrafeDirection extends TuningOpMode<Boolean> {
    String name;
    OctoQuadTuner.PodType podType;
    double customPodScalar;
    double headingScalar;
    int xPodPort, yPodPort;

    public OctoQuadStrafeDirection(String name, OctoQuadTuner.PodType podType, double customPodScalar, double headingScalar,
                                   int xPodPort, int yPodPort) {
        super("Strafe Direction Identification",
                "Determines if your strafe pod needs to be reversed. \n"
                        + "Push your robot to the left and then stop the Opmode",
                true);
        this.name = name;
        this.podType = podType;
        this.customPodScalar = customPodScalar;
        this.headingScalar = headingScalar;
        this.xPodPort = xPodPort;
        this.yPodPort = yPodPort;
    }

    @Override
    protected Boolean runTuningOpMode() {
        OctoQuadConfig config = new OctoQuadConfig(c -> {
            c.name.set(name);
            c.xPodPort.set(xPodPort);
            c.yPodPort.set(yPodPort);
            c.xPodDirection.set(OctoQuad.EncoderDirection.FORWARD);
            c.yPodDirection.set(OctoQuad.EncoderDirection.FORWARD);
            c.xPodOffset.set(0.0);
            c.yPodOffset.set(0.0);
            c.headingScalar.set(headingScalar);

            if (podType == OctoQuadTuner.PodType.CUSTOM) {
                c.ticksPerUnit.set(customPodScalar);
            } else {
                c.ticksPerUnit.set(podType == OctoQuadTuner.PodType.SWING_ARM ? OctoQuadTuner.SWING_ARM : OctoQuadTuner.FOUR_BAR);
            }
        });
        OctoQuadLocalizer localizer = new OctoQuadLocalizer(hardwareMap, config);
        localizer.setPose(new Pose(0, 0));
        waitForStart();
        while (!isStopRequested()) {
            localizer.update();
        }

        return localizer.pose().y() < 0;
    }
}

class OctoQuadOffsets extends TuningOpMode<List<Double>> {
    String name;
    OctoQuadTuner.PodType podType;
    double customPodScalar;
    boolean forwardPodReversed, strafePodReversed;
    double headingScalar;
    Pose previous;
    int xPodPort, yPodPort;

    public OctoQuadOffsets(String name, OctoQuadTuner.PodType podType, double customPodScalar, Boolean forwardPodReversed, Boolean strafePodReversed, double headingScalar,
                           int xPodPort, int yPodPort) {
        super("OctoQuadOffsets Identification",
                "Automatically identifies the offsets for your OctoQuad localizer. \n"
                        + "Spin your robot in place 180 degrees counterclockwise and then stop the Opmode",
                true);
        this.name = name;
        this.podType = podType;
        this.customPodScalar = customPodScalar;
        this.forwardPodReversed = forwardPodReversed;
        this.strafePodReversed = strafePodReversed;
        this.headingScalar = headingScalar;
        this.xPodPort = xPodPort;
        this.yPodPort = yPodPort;
    }

    @Override
    protected List<Double> runTuningOpMode() {
        OctoQuadConfig config = new OctoQuadConfig(c -> {
            c.name.set(name);
            c.xPodPort.set(xPodPort);
            c.yPodPort.set(yPodPort);
            c.xPodDirection.set(forwardPodReversed ? OctoQuad.EncoderDirection.REVERSE : OctoQuad.EncoderDirection.FORWARD);
            c.yPodDirection.set(strafePodReversed ? OctoQuad.EncoderDirection.REVERSE : OctoQuad.EncoderDirection.FORWARD);
            if (podType.equals(OctoQuadTuner.PodType.CUSTOM)) {
                c.encoderResolutionUnit.set(DistanceUnit.INCH);
                c.ticksPerUnit.set(customPodScalar);
            } else {
                c.encoderResolutionUnit.set(DistanceUnit.INCH);
                c.ticksPerUnit.set(podType == OctoQuadTuner.PodType.SWING_ARM ? OctoQuadTuner.SWING_ARM : OctoQuadTuner.FOUR_BAR);
            }
            c.xPodOffset.set(0.0);
            c.yPodOffset.set(0.0);
            c.headingScalar.set(headingScalar);
        });
        OctoQuadLocalizer localizer = new OctoQuadLocalizer(hardwareMap, config);
        localizer.setPose(new Pose(0, 0));
        localizer.update();

        waitForStart();

        while (!isStopRequested()) {
            previous = localizer.pose();
            localizer.update();
            telemetry.addData("heading", localizer.pose().heading());
            telemetry.update();
        }

        if (localizer.pose().x() != Pose.zero().x() || localizer.pose().y() != Pose.zero().y()) {
            previous =  localizer.pose();
        }

        return List.of(((-previous.y()) / 2.0), ((-previous.x()) / 2.0));
    }
}
