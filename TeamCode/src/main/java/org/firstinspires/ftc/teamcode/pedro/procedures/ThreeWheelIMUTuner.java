package org.firstinspires.ftc.teamcode.pedro.procedures;

import com.pedropathing.math.Pose;
import com.pedropathing.revhub.localizers.Encoder;
import com.pedropathing.revhub.localizers.RevHubIMU;
import com.pedropathing.revhub.localizers.ThreeWheelIMUConfig;
import com.pedropathing.revhub.localizers.ThreeWheelIMULocalizer;
import com.pedropathing.tuning.autotune.Inputs;
import com.pedropathing.tuning.autotune.Procedure;
import com.pedropathing.tuning.autotune.TuningOpMode;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.List;

public class ThreeWheelIMUTuner extends Procedure {

    private static String leftEncoderName = "lf";
    private static String rightEncoderName = "rr";
    private static String strafeEncoderName = "lr";
    private static String imuName = "imu";
    private static RevHubOrientationOnRobot.LogoFacingDirection logoDirection =
            RevHubOrientationOnRobot.LogoFacingDirection.UP;
    private static RevHubOrientationOnRobot.UsbFacingDirection usbDirection =
            RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD;

    public ThreeWheelIMUTuner() {
        super("Three Wheel + IMU Tuner", "Tune three odometry pods with an IMU");
    }

    @Override
    public void run() throws InterruptedException {
        Inputs setup = inputs("Encoder + IMU Setup",
                "Set encoder motor ports, IMU HardwareMap name, and Control Hub orientation.");
        Inputs.Field<String> leftEncoder = setup.s("Left Encoder Motor Name").withDefault("lf");
        Inputs.Field<String> rightEncoder = setup.s("Right Encoder Motor Name").withDefault("rr");
        Inputs.Field<String> strafeEncoder = setup.s("Strafe Encoder Motor Name").withDefault("lr");
        Inputs.Field<String> imu = setup.s("IMU HardwareMap Name").withDefault("imu");
        Inputs.Field<RevHubOrientationOnRobot.LogoFacingDirection> logo =
                setup.e("Logo Facing Direction", RevHubOrientationOnRobot.LogoFacingDirection.class)
                        .withDefault(RevHubOrientationOnRobot.LogoFacingDirection.UP);
        Inputs.Field<RevHubOrientationOnRobot.UsbFacingDirection> usb =
                setup.e("USB Facing Direction", RevHubOrientationOnRobot.UsbFacingDirection.class)
                        .withDefault(RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD);
        awaitInputs(setup);
        leftEncoderName = leftEncoder.get();
        rightEncoderName = rightEncoder.get();
        strafeEncoderName = strafeEncoder.get();
        imuName = imu.get();
        logoDirection = logo.get();
        usbDirection = usb.get();

        Inputs resolution = inputs("Encoder Resolution Identification",
                "Set a positive push distance in inches. Keep the robot straight during each push.");
        Inputs.Field<Double> distance = resolution.d("Distance").withDefault(48.0);
        awaitInputs(resolution);
        if (!(distance.get() > 0.0)) {
            abort("Enter a positive distance in inches.");
            return;
        }

        List<Double> left = measure("Left", distance.get());
        if (left == null) {
            return;
        }
        List<Double> right = measure("Right", distance.get());
        if (right == null) {
            return;
        }
        List<Double> strafe = measure("Strafe", distance.get());
        if (strafe == null) {
            return;
        }

        double leftTicksPerInch = left.get(0);
        double rightTicksPerInch = right.get(0);
        double strafeTicksPerInch = strafe.get(0);
        double forwardTicksPerInch = 2.0 / (1.0 / leftTicksPerInch + 1.0 / rightTicksPerInch);

        double forward = 1.0 / forwardTicksPerInch;
        double lateral = 1.0 / strafeTicksPerInch;

        List<Double> leftOffsets = runOpMode(new ThreeWheelIMUOffsets(
                true, forward, lateral, left.get(1), right.get(1), strafe.get(1)));
        if (leftOffsets == null) {
            abort("Left stage ended without parallel pod travel. Rotate 180 degrees CCW, then press Stop.");
            return;
        }
        List<Double> rightOffsets = runOpMode(new ThreeWheelIMUOffsets(
                false, forward, lateral, left.get(1), right.get(1), strafe.get(1)));

        if (rightOffsets == null) {
            abort("Right stage ended without parallel pod travel. Rotate 180 degrees CCW, then press Stop.");
            return;
        }
        ThreeWheelIMUConfig config = config(true, forward, lateral,
                left.get(1), right.get(1), strafe.get(1));
        config.leftPodY.set(leftOffsets.get(0));
        config.rightPodY.set(rightOffsets.get(0));
        config.turnTicksToRadians.set(forward);

        Double turn = runOpMode(new ThreeWheelIMUTurn(config));
        if (turn == null) {
            abort("Turn stage ended without positive rotation. Rotate 360 degrees CCW, then press Stop.");
            return;
        }
        double strafeX = (leftOffsets.get(1) + rightOffsets.get(1)) / 2.0 * turn / lateral;

        result("leftEncoderName", leftEncoderName);
        result("rightEncoderName", rightEncoderName);
        result("strafeEncoderName", strafeEncoderName);
        result("imuName", imuName);
        result("logoDirection", logoDirection);
        result("usbDirection", usbDirection);
        result("leftPodY", leftOffsets.get(0));
        result("rightPodY", rightOffsets.get(0));
        result("strafePodX", strafeX);
        result("leftTicksPerInch", leftTicksPerInch);
        result("rightTicksPerInch", rightTicksPerInch);
        result("forwardTicksPerInch", forwardTicksPerInch);
        result("strafeTicksPerInch", strafeTicksPerInch);
        result("forwardTicksToInches", forward);
        result("strafeTicksToInches", lateral);
        result("turnTicksToRadians", turn);
        result("leftEncoderDirection", direction(left.get(1)));
        result("rightEncoderDirection", direction(right.get(1)));
        result("strafeEncoderDirection", direction(strafe.get(1)));

        code(Language.JAVA,
                "public static ThreeWheelIMUConfig localizerConfig = new ThreeWheelIMUConfig(c -> {\n" +
                        "    c.leftEncoderName.set(\"" + leftEncoderName + "\");\n" +
                        "    c.rightEncoderName.set(\"" + rightEncoderName + "\");\n" +
                        "    c.strafeEncoderName.set(\"" + strafeEncoderName + "\");\n" +
                        "    c.imuName.set(\"" + imuName + "\");\n" +
                        "    c.imu.set(new RevHubIMU(new RevHubOrientationOnRobot(\n" +
                        "            RevHubOrientationOnRobot.LogoFacingDirection." + logoDirection + ",\n" +
                        "            RevHubOrientationOnRobot.UsbFacingDirection." + usbDirection + "\n" +
                        "    )));\n" +
                        "    c.leftPodY.set(" + leftOffsets.get(0) + ");\n" +
                        "    c.rightPodY.set(" + rightOffsets.get(0) + ");\n" +
                        "    c.strafePodX.set(" + strafeX + ");\n" +
                        "    c.forwardTicksToInches.set(" + forward + ");\n" +
                        "    c.strafeTicksToInches.set(" + lateral + ");\n" +
                        "    c.turnTicksToRadians.set(" + turn + ");\n" +
                        "    c.leftEncoderDirection.set(" + direction(left.get(1)) + ");\n" +
                        "    c.rightEncoderDirection.set(" + direction(right.get(1)) + ");\n" +
                        "    c.strafeEncoderDirection.set(" + direction(strafe.get(1)) + ");\n" +
                        "});");
    }

    private List<Double> measure(String pod, double distance) throws InterruptedException {
        List<Double> measured = runOpMode(new ThreeWheelIMUResolution(pod, distance));
        if (measured == null) {
            abort(pod + " stage ended without a nonzero measurement. Check the displayed ticks, complete the push, then press Stop.");
            return null;
        }
        return measured;
    }

    static String direction(double direction) {
        return direction == Encoder.REVERSE ? "Encoder.REVERSE" : "Encoder.FORWARD";
    }

    static ThreeWheelIMUConfig config(boolean left, double forward, double strafe,
                                      double leftDirection, double rightDirection, double strafeDirection) {
        return new ThreeWheelIMUConfig(c -> {
            c.leftEncoderName.set(leftEncoderName);
            c.rightEncoderName.set(rightEncoderName);
            c.strafeEncoderName.set(strafeEncoderName);
            c.imuName.set(imuName);
            c.imu.set(new RevHubIMU(new RevHubOrientationOnRobot(logoDirection, usbDirection)));            c.leftPodY.set(left ? 0.0 : 1.0);
            c.rightPodY.set(left ? -1.0 : 0.0);
            c.strafePodX.set(0.0);
            c.forwardTicksToInches.set(forward);
            c.strafeTicksToInches.set(strafe);
            c.turnTicksToRadians.set(0.0);
            c.leftEncoderDirection.set(leftDirection);
            c.rightEncoderDirection.set(rightDirection);
            c.strafeEncoderDirection.set(strafeDirection);
        });
    }

    static ThreeWheelIMULocalizer localizer(HardwareMap map, ThreeWheelIMUConfig config) {
        for (LynxModule hub : map.getAll(LynxModule.class)) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        for (String name : new String[]{"lf", "lr", "rf", "rr"}) {
            DcMotorEx motor = map.get(DcMotorEx.class, name);
            motor.setPower(0);
            motor.setDirection(name.equals("lf") || name.equals("lr")
                    ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }
        return new ThreeWheelIMULocalizer(map, config);
    }
}

class ThreeWheelIMUResolution extends TuningOpMode<List<Double>> {

    String pod;
    double distance;

    ThreeWheelIMUResolution(String pod, double distance) {
        super(pod + " Encoder Resolution and Direction",
                "After Start, push the robot " + (pod.equals("Strafe") ? "left " : "forward ") +
                        distance + " inches exactly without turning. Stop moving, press Stop to save this measurement.", true);
        this.pod = pod;
        this.distance = distance;
    }

    @Override
    protected List<Double> runTuningOpMode() {
        ThreeWheelIMUConfig config = ThreeWheelIMUTuner.config(!pod.equals("Right"), 1.0, 1.0,
                Encoder.FORWARD, Encoder.FORWARD, Encoder.FORWARD);
        ThreeWheelIMULocalizer localizer = ThreeWheelIMUTuner.localizer(hardwareMap, config);
        localizer.setPose(new Pose(0, 0));
        Pose position = null;

        waitForStart();
        while (!isStopRequested()) {
            localizer.update();
            position = localizer.pose();
        }

        if (position == null) {
            return null;
        }
        double movement = pod.equals("Strafe") ? position.y() : position.x();
        if (movement == 0.0) {
            return null;
        }
        return List.of(Math.abs(movement / distance), movement < 0 ? Encoder.REVERSE : Encoder.FORWARD);
    }
}

class ThreeWheelIMUOffsets extends TuningOpMode<List<Double>> {

    boolean left;
    double forward;
    double strafe;
    double leftDirection;
    double rightDirection;
    double strafeDirection;

    ThreeWheelIMUOffsets(boolean left, double forward, double strafe,
                         double leftDirection, double rightDirection, double strafeDirection) {
        super((left ? "Left" : "Right") + " Pod Offset Identification",
                "After Start, rotate exactly 180 degrees counterclockwise about the robot center. " +
                        "Keep that center fixed. Stop moving, press Stop to save this measurement.", true);
        this.left = left;
        this.forward = forward;
        this.strafe = strafe;
        this.leftDirection = leftDirection;
        this.rightDirection = rightDirection;
        this.strafeDirection = strafeDirection;
    }

    @Override
    protected List<Double> runTuningOpMode() {
        ThreeWheelIMUConfig config = ThreeWheelIMUTuner.config(left, forward, strafe,
                leftDirection, rightDirection, strafeDirection);
        boolean previousUseIMU = ThreeWheelIMULocalizer.useIMU;
        ThreeWheelIMULocalizer.useIMU = false;
        try {
            ThreeWheelIMULocalizer localizer = ThreeWheelIMUTuner.localizer(hardwareMap, config);
            localizer.setPose(new Pose(0, 0));
            localizer.update();
            Pose position = null;

            waitForStart();
            while (!isStopRequested()) {
                localizer.update();
                position = localizer.pose();
            }

            if (position == null) {
                return null;
            }
            return List.of(-position.x() / Math.PI, position.y() / Math.PI);
        } finally {
            ThreeWheelIMULocalizer.useIMU = previousUseIMU;
        }
    }
}

class ThreeWheelIMUTurn extends TuningOpMode<Double> {

    ThreeWheelIMUConfig config;

    ThreeWheelIMUTurn(ThreeWheelIMUConfig config) {
        super("Turn Multiplier Identification",
                "After Start, rotate exactly 360 degrees counterclockwise. " +
                        "Stop moving, press Stop to save this measurement.", true);
        this.config = config;
    }

    @Override
    protected Double runTuningOpMode() {
        boolean previousUseIMU = ThreeWheelIMULocalizer.useIMU;
        ThreeWheelIMULocalizer.useIMU = false;
        try {
            ThreeWheelIMULocalizer localizer = ThreeWheelIMUTuner.localizer(hardwareMap, config);
            localizer.setPose(new Pose(0, 0));
            localizer.update();
            double startHeading = localizer.getTotalHeading();
            Double heading = null;

            waitForStart();
            while (!isStopRequested()) {
                localizer.update();
                heading = localizer.getTotalHeading();
            }

            if (heading == null || heading <= startHeading) {
                return null;
            }
            return config.turnTicksToRadians.get() * (2.0 * Math.PI) / (heading - startHeading);
        } finally {
            ThreeWheelIMULocalizer.useIMU = previousUseIMU;
        }
    }
}


