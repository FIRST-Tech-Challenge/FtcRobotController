package org.firstinspires.ftc.teamcode.simloopexample;

import org.horizon36596.simloop.config.DrivetrainSimConfig;
import org.horizon36596.simloop.config.MechanismSimConfig;
import org.horizon36596.simloop.config.PositionalServoSimConfig;
import org.horizon36596.simloop.config.SimRobotConfig;
import org.horizon36596.simloop.fakehardware.FakeExternalEncoder;
import org.horizon36596.simloop.fakehardware.FakeHardwareMap;
import org.horizon36596.simloop.fakehardware.FakeMotor;
import org.horizon36596.simloop.fakehardware.FakeServo;
import org.horizon36596.simloop.field.FieldFrameTransform;
import org.horizon36596.simloop.plant.MecanumDrivePlant;
import org.horizon36596.simloop.plant.Mechanism1DofPlant;
import org.horizon36596.simloop.plant.PositionalServoPlant;
import org.psilynx.psikit.core.Logger;
import org.psilynx.psikit.core.wpi.math.Pose2d;
import org.psilynx.psikit.core.wpi.math.Rotation2d;

/**
 * The simulated {@link ExampleRobot}: fake devices under the names the robot's own subsystems look up,
 * a plant behind each one, and the physical numbers that describe the machine.
 *
 * <p><b>This is the file your team writes once and then stops thinking about.</b> The subsystems in
 * {@code simloop-starter/robot/} know nothing about any of this; every test in the project builds one of
 * these and drives the robot through it, so a new mechanism is added here once rather than in each test.
 *
 * <h2>The order a tick happens in</h2>
 *
 * <p>Getting this wrong is the one mistake that produces a plausible-looking wrong answer rather than an
 * error, so it is written out. Each tick:
 *
 * <ol>
 *   <li><b>Robot code runs</b> - {@code robot.periodic()}, or whatever the scenario is testing. It reads
 *       sensors and writes motor powers, exactly as it would in an OpMode loop.</li>
 *   <li><b>The plants advance</b>, reading the powers that robot code just wrote.</li>
 *   <li><b>The sensors are told what the plants did</b> - {@link #advancePhysics(double)} pushes the
 *       slide plant's position into the slide encoder. This is the step that makes the next tick's robot
 *       code see the truth instead of its own command.</li>
 *   <li><b>The devices tick</b> - {@code hardwareMap.updateAll}, which is where each fake derives what it
 *       derives from elapsed time, such as the encoder's velocity.</li>
 * </ol>
 *
 * <p>A test therefore calls {@code robot.periodic()} and then {@link #advancePhysics(double)}, in that
 * order, once per tick.
 *
 * <h2>Where these numbers come from</h2>
 *
 * <p>They are invented - plausible round figures for the invented robot, not measurements of anything.
 * On your own robot every one of them is measured or characterised, and the value of doing that is that
 * a simulation built on measured numbers can be wrong in a way you can see.
 */
public final class ExampleRobotSim {

    /** Inches in one metre. AdvantageScope's field is metres; SimLoop's plants work in inches. */
    private static final double INCHES_PER_METRE = 39.37007874015748;

    /** Encoder counts per revolution of a 435 rpm gearmotor's output shaft. */
    private static final double DRIVE_TICKS_PER_REV = 384.5;

    /** Free-speed shaft rate of a drive motor, in encoder ticks per second. */
    private static final double DRIVE_MAX_TICKS_PER_SECOND = 2400.0;

    /** Free-speed shaft rate of the slide motor, in encoder ticks per second. */
    private static final double SLIDE_MAX_TICKS_PER_SECOND = 2000.0;

    /** Counts per revolution of the through-bore encoder on the slide. */
    private static final double SLIDE_ENCODER_TICKS_PER_REV = 8192.0;

    /**
     * First-order rate constant for how quickly a motor's shaft speed follows its commanded power, in
     * reciprocal seconds. 8.0 is a shaft that is most of the way there in about an eighth of a second.
     */
    private static final double MOTOR_RATE_CONSTANT = 8.0;

    /** The slide's physical behaviour. */
    private static final MechanismSimConfig SLIDE = new MechanismSimConfig() {
        /** Seconds for the carriage speed to follow a change in commanded power. */
        @Override public double timeConstant() { return 0.12; }
        /** Carriage speed at full power with no load fighting it, in inches per second. */
        @Override public double maxSpeed() { return 30.0; }
        /** Bottom of travel, in inches. */
        @Override public double minPosition() { return 0.0; }
        /** Top of travel, in inches. */
        @Override public double maxPosition() { return 24.0; }
        /**
         * Power the carriage's own weight costs, unitless. This is the number
         * {@code ExampleSlide}'s gravity feedforward has to cancel, and the reason it has one.
         */
        @Override public double gravityHoldPowerFraction() { return 0.08; }
    };

    /**
     * The claw's physical behaviour. Its position units are the servo's own command units, so the
     * command-to-position map below is the identity and a position reads as a servo command.
     */
    private static final PositionalServoSimConfig CLAW = new PositionalServoSimConfig() {
        /** Seconds for the horn speed to follow a change in commanded position. */
        @Override public double timeConstant() { return 0.05; }
        /** Full-speed horn travel, in command units per second - a full sweep in about 0.6 s. */
        @Override public double maxSpeed() { return 1.6; }
        /** One end of the horn's travel, in command units. */
        @Override public double minPosition() { return 0.0; }
        /** The other end of the horn's travel, in command units. */
        @Override public double maxPosition() { return 1.0; }
        /** Command 0.0 puts the horn here. Identity map: these are the same units. */
        @Override public double positionAtCommandZero() { return 0.0; }
        /** Command 1.0 puts the horn here. */
        @Override public double positionAtCommandOne() { return 1.0; }
        /**
         * How far off the commanded position the horn is allowed to park, in command units. A real
         * servo's own comparator stops driving inside this band, which is why a servo joint keeps a
         * small steady-state error that no amount of waiting removes.
         */
        @Override public double commandDeadband() { return 0.01; }
    };

    /** The chassis geometry, and the field it drives on. */
    private static final DrivetrainSimConfig DRIVETRAIN = new DrivetrainSimConfig() {
        /** Left-right wheel centre-to-centre distance, in inches. */
        @Override public double trackWidth() { return 13.0; }
        /** Front-back wheel centre-to-centre distance, in inches. */
        @Override public double wheelBase() { return 12.0; }
        /** Mecanum wheel radius, in inches. */
        @Override public double wheelRadius() { return 1.89; }
        /** Encoder counts per inch the wheel rolls. */
        @Override public double ticksPerInch() { return 45.0; }
        /**
         * Which way each motor is BOLTED IN, in {@code [FL, FR, BL, BR]} order - a fact about the metal,
         * and the counterpart of the {@code Direction.REVERSE} that {@link ExampleDrive} sets on the
         * right-hand pair in software.
         *
         * <p>This one is worth stopping on, because leaving it at its all-{@code +1} default is a
         * mistake that looks like nothing. The default describes a robot whose four motors are mounted
         * identically, which no mecanum chassis is; against that robot, {@code ExampleDrive}'s software
         * reversal is uncancelled and the two sides fight each other. Measured: one second of "drive
         * forward" then moves the chassis 0.0 inches in x, 0.0 in y, and turns it -2.50 radians. It
         * translates nowhere and spins instead, which is exactly what the same mistake does on the
         * field. It is what this example did on its first run.
         *
         * <p>The product of the two signs is what moves the robot. Get either one wrong on its own and
         * the simulation misbehaves for the same reason the real robot would.
         */
        @Override public double[] wheelMountingSigns() { return new double[] {1.0, -1.0, 1.0, -1.0}; }
        /** Free-speed shaft rate of a drive motor, in ticks per second. */
        @Override public double maxVelocityTicksPerSecond() { return DRIVE_MAX_TICKS_PER_SECOND; }
        /** How quickly a wheel reaches its commanded rate, in reciprocal seconds. */
        @Override public double maxAccel() { return MOTOR_RATE_CONSTANT; }
        /** Half the field, in inches - an FTC field is 12 ft square. */
        @Override public double fieldHalfWidth() { return 72.0; }
        /** Half the field, in inches. */
        @Override public double fieldHalfHeight() { return 72.0; }
    };

    /**
     * The robot-level configuration a scenario runs under. SimLoop needs one even for a scenario that
     * never drives, because a run is named after the robot it ran on.
     */
    public static final SimRobotConfig ROBOT_CONFIG = new SimRobotConfig() {
        /** The four drive motors, in the order the plant reads them. */
        @Override public String[] driveMotorNames() { return ExampleDrive.MOTOR_NAMES; }
        /** The odometry device's configuration name. This robot has none, so the name is unused. */
        @Override public String odometryName() { return "odo"; }
        /** The chassis geometry. */
        @Override public DrivetrainSimConfig drivetrain() { return DRIVETRAIN; }
    };

    /** The hardware map the robot's subsystems resolve everything from. */
    public final FakeHardwareMap hardwareMap = new FakeHardwareMap();

    /** The four drive motors, in {@code [frontLeft, frontRight, backLeft, backRight]} order. */
    public final FakeMotor[] driveMotors = new FakeMotor[4];

    /** The motor that drives the slide carriage. */
    public final FakeMotor slideMotor =
            new FakeMotor(SLIDE_MAX_TICKS_PER_SECOND, MOTOR_RATE_CONSTANT, SLIDE_ENCODER_TICKS_PER_REV);

    /** The through-bore encoder that measures the slide carriage. */
    public final FakeExternalEncoder slideEncoder =
            new FakeExternalEncoder(SLIDE_MAX_TICKS_PER_SECOND, SLIDE_ENCODER_TICKS_PER_REV);

    /** The claw servo. */
    public final FakeServo clawServo = new FakeServo();

    /** The chassis physics. */
    public final MecanumDrivePlant drivePlant;

    /** The slide physics. */
    public final Mechanism1DofPlant slidePlant;

    /** The claw physics - what the jaws are really doing, which the robot cannot see. */
    public final PositionalServoPlant clawPlant;

    /** The robot under test. The same class an OpMode would build. */
    public final ExampleRobot robot;

    /** Registers every fake device, builds the plants behind them, and constructs the robot. */
    public ExampleRobotSim() {
        for (int i = 0; i < driveMotors.length; i++) {
            driveMotors[i] = new FakeMotor(
                    DRIVE_MAX_TICKS_PER_SECOND, MOTOR_RATE_CONSTANT, DRIVE_TICKS_PER_REV);
            hardwareMap.register(ExampleDrive.MOTOR_NAMES[i], driveMotors[i]);
        }
        hardwareMap.register("slideMotor", slideMotor);
        hardwareMap.register("slideEncoder", slideEncoder);
        hardwareMap.register("claw", clawServo);

        drivePlant = new MecanumDrivePlant(driveMotors, DRIVETRAIN);
        slidePlant = new Mechanism1DofPlant(slideMotor, SLIDE);
        clawPlant = new PositionalServoPlant(clawServo, CLAW);

        // Built last, and from the same hardware map an OpMode would be handed. Everything above this
        // line is the robot's wiring; nothing below it knows the wiring is fake.
        robot = new ExampleRobot(hardwareMap);
    }

    /**
     * Logs the chassis pose where AdvantageScope's 2D and 3D field tabs look for it: a WPILib
     * {@link Pose2d} struct, in metres and radians.
     *
     * <p>The drivetrain plant reports the <b>localizer frame</b> ({@code +x} forward, {@code +y} left,
     * heading from localizer {@code +x}). AdvantageScope's FTC fields use the field frame: origin at the
     * centre of the 12 ft field, {@code +X} to the robot's right when it faces up the field, {@code +Y} up
     * the field, heading counter-clockwise from {@code +X}. That is the same frame SimLoop's
     * {@link FieldFrameTransform} produces, so the rotation is the only conversion; there is no
     * translation, because both frames put the origin at the centre of the field.
     *
     * <p>Only the units differ: SimLoop works in inches, AdvantageScope in metres.
     *
     * @param key the log key to write the pose under, for example {@code "View/robotPose"}
     */
    public void logPose(String key) {
        double fieldXInches = FieldFrameTransform.fieldXFromLocalizer(drivePlant.getX(), drivePlant.getY());
        double fieldYInches = FieldFrameTransform.fieldYFromLocalizer(drivePlant.getX(), drivePlant.getY());
        double fieldHeading = FieldFrameTransform.fieldHeadingFromLocalizer(drivePlant.getHeading());

        double xMetres = fieldXInches / INCHES_PER_METRE;
        double yMetres = fieldYInches / INCHES_PER_METRE;

        Logger.recordOutput(key, Pose2d.struct,
                new Pose2d(xMetres, yMetres, new Rotation2d(fieldHeading)));
    }

    /**
     * Advances the physical world by one tick, after robot code has written its motor powers.
     *
     * <p>Call this once per tick, immediately after the robot code for that tick - see the class
     * comment for why the order matters.
     *
     * @param deltaTime the length of this tick, in seconds
     */
    public void advancePhysics(double deltaTime) {
        drivePlant.update(deltaTime);
        slidePlant.update(deltaTime);
        clawPlant.update(deltaTime);

        // Tell the slide motor how fast its shaft is REALLY turning. Without this the motor keeps
        // modelling a happily spinning shaft while the carriage sits on an end stop, so anything that
        // reads motor current - a stall detector, a re-zero routine - reads a number the mechanism does
        // not agree with.
        slideMotor.setMeasuredShaftSpeedFraction(slidePlant.getVelocityFractionOfMaxSpeed());

        // Push the carriage's true position into the encoder the subsystem reads. This is the whole
        // reason the slide has a separate encoder device: the motor integrates its own position from
        // commanded power and knows nothing about the end stops, and the carriage encoder does.
        slideEncoder.setEncoderPosition(slidePlant.getPosition() * ExampleSlide.TICKS_PER_INCH);

        hardwareMap.updateAll(deltaTime);
    }
}
