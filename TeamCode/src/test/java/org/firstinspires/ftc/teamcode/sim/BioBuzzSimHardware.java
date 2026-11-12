package org.firstinspires.ftc.teamcode.sim;

import org.horizon36596.simloop.fakehardware.FakeHardwareMap;
import org.horizon36596.simloop.fakehardware.FakeMotor;
import org.horizon36596.simloop.fakehardware.FakeServo;
import org.horizon36596.simloop.fakehardware.FakeCRServo;
import org.horizon36596.simloop.fakehardware.FakeAnalogInput;
import org.horizon36596.simloop.fakehardware.FakeDigitalChannel;
import org.horizon36596.simloop.fakehardware.FakeVoltageSensor;
import org.horizon36596.simloop.fakehardware.FakeGoBildaPinpoint;
import org.horizon36596.simloop.fakehardware.FakeLimelight3A;
import org.horizon36596.simloop.vision.AprilTagLayout;
import org.horizon36596.simloop.vision.CameraMount;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver.EncoderDirection;

/** Mechanism-only fixtures: an unrelated lookup must fail, rather than silently supplying hardware. */
final class BioBuzzSimHardware {
    final FakeHardwareMap map = new FakeHardwareMap();
    final FakeMotor[] wheels = new FakeMotor[4]; // FL, FR, BL, BR
    FakeServo gate;
    FakeMotor intake, transfer1, transfer2, flywheel, encoder;
    FakeServo yaw1, yaw2, hood, depollinator;
    FakeServo ledIntake, ledShooter, ledStatus;
    FakeCRServo slideServo;
    FakeAnalogInput slideEncoder;
    FakeServo claw;
    FakeDigitalChannel distance;
    FakeGoBildaPinpoint pinpoint;
    FakeLimelight3A camera;
    TargetNormalCameraAdapter cameraAdapter;

    private FakeMotor motor(String name) {
        // Preserve the generic mechanism fixture; drivetrain shaft speed is specified separately.
        FakeMotor motor = new FakeMotor(28 * 5550.0 / 60, 10, 28);
        map.register(name, motor);
        return motor;
    }

    private FakeServo servo(String name) {
        FakeServo servo = new FakeServo();
        map.register(name, servo);
        return servo;
    }

    BioBuzzSimHardware withIntake() { intake = motor("intake_motor"); return this; }
    BioBuzzSimHardware withLights() {
        ledIntake = servo("led_intake"); ledShooter = servo("led_shooter"); ledStatus = servo("led_status");
        return this;
    }
    BioBuzzSimHardware withTransfer() {
        if (gate == null) gate = servo("gate_servo");
        transfer1 = motor("transfer_motor_1"); transfer2 = motor("transfer_motor_2");
        distance = new FakeDigitalChannel(); map.register("distance_sensor", distance);
        distance.setSimulatedInputState(false);
        return this;
    }
    BioBuzzSimHardware withDepollinator() {
        depollinator = servo("depollinator"); return this;
    }
    /** Slide CR servo, its analog feedback and the claw. The model starts the servo shaft at
     * 300 deg so the first raise crosses the analog 360/0 wrap. */
    static final double SLIDE_START_SHAFT_DEG = 300;
    BioBuzzSimHardware withSlide() {
        slideServo = new FakeCRServo(); map.register("slide_servo", slideServo);
        slideEncoder = new FakeAnalogInput(); map.register("slide_encoder", slideEncoder);
        slideEncoder.setVoltage(SLIDE_START_SHAFT_DEG / 360 * slideEncoder.getMaxVoltage());
        claw = servo("claw_servo");
        return this;
    }
    BioBuzzSimHardware withTurret() {
        yaw1 = servo("turret_servo_1"); yaw2 = servo("turret_servo_2"); hood = servo("turret_pitch");
        // Independent team observation: 5400 RPM at 14 V, 28 ticks/rev, 1:1 drive.
        // Response rate remains synthetic; no calibrated shot/load dynamics.
        flywheel = new FakeMotor(5400.0 / 60 * 28, 10, 28);
        map.register("turret_motor", flywheel);
        // Real through-bore quadrature is plugged into transfer motor 1's encoder port.
        // Register only the installed name, exercising the production fallback lookup.
        if (transfer1 == null) transfer1 = motor("transfer_motor_1");
        encoder = transfer1;
        map.register("Control Hub", new FakeVoltageSensor(14.0));
        return this;
    }
    BioBuzzSimHardware withPinpoint() {
        pinpoint = new FakeGoBildaPinpoint();
        pinpoint.setPhysicalPodMount(32, -74, EncoderDirection.REVERSED,
                EncoderDirection.REVERSED, 2000 / (32 * Math.PI));
        pinpoint.attachChassisAt(0, 0, 0);
        map.register("pinpoint", pinpoint);
        return this;
    }
    /** Physical-convention camera: tag +Z into the print and upside-down underside stickers (see adapter). */
    BioBuzzSimHardware withCamera(AprilTagLayout layout) {
        return withCamera(layout, true);
    }
    /** false additionally inverts the tag normal, which every OpMode must reject. */
    BioBuzzSimHardware withCamera(AprilTagLayout layout, boolean physicalTagConvention) {
        if (camera != null) map.remove(null, "limelight", camera);
        if (cameraAdapter != null) map.remove(null, "limelight", cameraAdapter);
        // Camera lives on the rotating turret. Model supplies the pivot pose, not the chassis pose.
        camera = FakeLimelight3A.builder(layout,
                new CameraMount(-99.53073974 / 25.4, 0, org.firstinspires.ftc.teamcode.ShootingConfiguration.CAMERA_HEIGHT_MM / 25.4, 0,
                        -Math.toRadians(org.firstinspires.ftc.teamcode.ShootingConfiguration.CAMERA_PITCH_UP_DEG)), 54, 41)
                .rangeInches(2, 200).frameRateHz(30).latencyMs(15, 20).build();
        cameraAdapter = new TargetNormalCameraAdapter(camera, !physicalTagConvention);
        map.register("limelight", cameraAdapter);
        return this;
    }
    static BioBuzzSimHardware fullRobot() {
        return drivetrain().withIntake().withTransfer().withDepollinator().withSlide().withTurret()
                .withPinpoint().withCamera(AprilTagLayout.empty());
    }

    /** Raised-tag start Y (in). With the measured 55-deg Limelight the tags leave the image beyond
     * about 52 in; 40 in keeps them well in view, near the real robot's 2026-11-05 test distance. */
    static final double RAISED_TAG_VIEW_Y_INCHES = -40;

    /** SimLoop tilt-up of the sticker normal: the underside sticker (30 deg from horizontal)
     * faces 60 deg below horizontal, out of the hive toward the robot, as measured on the field 2026-11-05. */
    static final double RAISED_TAG_TILT_UP_RAD =
            -Math.toRadians(90 - org.firstinspires.ftc.teamcode.HiveTagGeometry.RAISED_TAG_PLANE_DEG);

    static AprilTagLayout raisedAudienceTags() {
        AprilTagLayout.Builder layout = AprilTagLayout.builder();
        // TU06 nominal map: one raised cell per alliance; never both states of the same hive.
        double[] offsets = {-6.5, -2.75, 2.75, 6.5};
        for (int side = 0; side < 2; side++) for (int i = 0; i < 4; i++) {
            // Audience IDs decrease with X (upside-down underside stickers; physical frame 2026-11-05).
            layout.addTiltedTag(34 + 4 * side + i, (side == 0 ? -12.75 : 12.75) - offsets[i],
                    -org.firstinspires.ftc.teamcode.HiveTagGeometry.RAISED_TAG_ROW_Y_METERS / .0254,
                    org.firstinspires.ftc.teamcode.HiveTagGeometry.DRAWING_TAG_HEIGHT_METERS / .0254,
                    -Math.PI / 2, RAISED_TAG_TILT_UP_RAD, 3.25);
        }
        return layout.build();
    }

    /** Explicit alternate hive state for corner rehome: red far and blue audience are raised. */
    static AprilTagLayout raisedCornerRehomeTags() {
        AprilTagLayout.Builder layout = AprilTagLayout.builder();
        double[] offsets = {-6.5, -2.75, 2.75, 6.5};
        for (int i = 0; i < 4; i++) {
            double y = org.firstinspires.ftc.teamcode.HiveTagGeometry.RAISED_TAG_ROW_Y_METERS / .0254;
            double z = org.firstinspires.ftc.teamcode.HiveTagGeometry.DRAWING_TAG_HEIGHT_METERS / .0254;
            // Red far IDs increase with X (tag 30 camera-right); blue audience IDs decrease with X.
            layout.addTiltedTag(30+i,-12.75+offsets[i],y,z,Math.PI/2,RAISED_TAG_TILT_UP_RAD,3.25);
            layout.addTiltedTag(38+i,12.75-offsets[i],-y,z,-Math.PI/2,RAISED_TAG_TILT_UP_RAD,3.25);
        }
        return layout.build();
    }

    static BioBuzzSimHardware forOpMode(String name) {
        switch (name) {
            case "DrivetrainTestOpMode": return drivetrain();
            case "GateTestOpMode": return gate();
            case "IntakeTestOpMode": return new BioBuzzSimHardware().withIntake();
            case "LightsTestOpMode": return new BioBuzzSimHardware().withLights();
            case "TransferTestOpMode": return new BioBuzzSimHardware().withTransfer();
            case "DepollinatorTestOpMode": return new BioBuzzSimHardware().withDepollinator();
            case "LinearSlideTestOpMode": return new BioBuzzSimHardware().withSlide();
            case "TurretTestOpMode": return new BioBuzzSimHardware().withTurret();
            case "PinpointTestOpMode": return new BioBuzzSimHardware().withPinpoint();
            case "ShotCalibrationOpMode":
            case "IntakeShooterTestOpMode": return new BioBuzzSimHardware().withIntake().withTransfer().withTurret();
            case "RedMecanumTeleOp": case "BlueMecanumTeleOp":
            case "AutoAimTestOpMode": case "LimelightTagTestOpMode": case "GoalStateTestOpMode":
            case "PedroDriveTestOpMode": return fullRobot();
            default: throw new IllegalArgumentException("Unsupported review OpMode: " + name);
        }
    }

    static BioBuzzSimHardware drivetrain(String... names) {
        BioBuzzSimHardware hardware = new BioBuzzSimHardware();
        for (int i = 0; i < 4; i++) {
            hardware.wheels[i] = new FakeMotor(BioBuzzSimModel.FREE_SPEED_TICKS_PER_SECOND,
                    BioBuzzSimModel.RESPONSE_RATE_PER_SECOND, BioBuzzSimModel.MOTOR_TICKS_PER_REV);
            hardware.map.register(names[i], hardware.wheels[i]);
        }
        return hardware;
    }

    static BioBuzzSimHardware drivetrain() {
        return drivetrain("leftfront", "rightfront", "leftback", "rightback");
    }

    static BioBuzzSimHardware gate() {
        BioBuzzSimHardware hardware = new BioBuzzSimHardware();
        hardware.gate = new FakeServo();
        hardware.map.register("gate_servo", hardware.gate);
        return hardware;
    }
}
