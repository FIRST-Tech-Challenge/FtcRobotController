package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.HashMap;
import java.util.Map;
import java.util.Objects;


/** Stores the Robot's hardware and position.
 *  Also has a "desired state" for mechanism driving.
 */
public class Robot {
    // Robot desired states.

    public enum SlidesState {RETRACTED, LOW, MEDIUM, HIGH, UNREADY, MOVE_UP, MOVE_DOWN, STOPPED, FIRST_STACK_CONE, SECOND_STACK_CONE}
    public enum ParkingPosition {INSIDE, MIDDLE, OUTSIDE}
    public enum ClawRotatorState {FRONT, SIDE, REAR}
    public enum ClawState {OPEN, CLOSED}
    public enum SecondaryClawState {OPEN, CLOSED}
    public enum SecondaryClawRotatorState {DOWN, UP}
    public enum ClawLimitSwitchServoState {HIGH, LOW}
    public enum SecondarySlidesState {RETRACTED, EXTENDED}

    public static SlidesState desiredSlidesState = SlidesState.UNREADY;
    public ClawRotatorState desiredClawRotatorState;
    public ClawLimitSwitchServoState desiredClawLimitSwitchState;
    public ClawState desiredClawState;
    public SecondaryClawState desiredSecondaryClawState;
    public SecondaryClawRotatorState desiredSecondaryClawRotatorState;
    public ClawLimitSwitchServoState desiredClawLimitSwitchServoState;

    public boolean previousSlidesLimitSwitchState = false;
    public boolean previousClawLimitSwitchState = false;

    enum MovementMode {NORMAL, FINE, ULTRA_FINE}
    MovementMode movementMode = MovementMode.NORMAL;
    boolean wheelSpeedAdjustment = false;

    HashMap<RobotConfig.DriveMotors, DcMotor> driveMotors = new HashMap<RobotConfig.DriveMotors, DcMotor>();

    // Hardware
    public DcMotor slidesMotor1, slidesMotor2, secondarySlidesMotor;
    public Servo clawRotator, claw, clawLimitSwitchServo, secondaryClaw, secondaryClawRotator;
    public TouchSensor slidesLimitSwitch;
    public TouchSensor clawLimitSwitch;
    public DistanceSensor clawDistanceSensor;



    // Other
    public Telemetry telemetry;
    public ElapsedTime elapsedTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    public PositionManager positionManager;

    public Robot(HardwareMap hardwareMap, Telemetry telemetry, ElapsedTime elapsedTime) {
        this.telemetry = telemetry;
        this.elapsedTime = elapsedTime;
        positionManager = new PositionManager(hardwareMap, telemetry);

        desiredClawRotatorState = ClawRotatorState.FRONT;
        desiredClawState = ClawState.CLOSED;

        slidesMotor1 = hardwareMap.get(DcMotor.class, RobotConfig.MotorNames.get(RobotConfig.Motors.SLIDES_MOTOR_1));
        slidesMotor2 = hardwareMap.get(DcMotor.class, RobotConfig.MotorNames.get(RobotConfig.Motors.SLIDES_MOTOR_2));
        secondarySlidesMotor = hardwareMap.get(DcMotor.class, RobotConfig.MotorNames.get(RobotConfig.Motors.SECONDARY_SLIDES_MOTOR));
        clawRotator = hardwareMap.get(Servo.class, RobotConfig.ServoNames.get(RobotConfig.Servos.CLAW_ROTATOR));
        claw = hardwareMap.get(Servo.class, RobotConfig.ServoNames.get(RobotConfig.Servos.CLAW));
        clawLimitSwitchServo = hardwareMap.get(Servo.class, RobotConfig.ServoNames.get(RobotConfig.Servos.CLAW_LIMIT_SWITCH_SERVO));
        secondaryClaw = hardwareMap.get(Servo.class, RobotConfig.ServoNames.get(RobotConfig.Servos.SECONDARY_CLAW));
        secondaryClawRotator = hardwareMap.get(Servo.class, RobotConfig.ServoNames.get(RobotConfig.Servos.SECONDARY_CLAW_ROTATOR));


        slidesLimitSwitch = hardwareMap.get(TouchSensor.class, RobotConfig.SwitchNames.get(RobotConfig.Switches.SLIDES_LIMIT));
        clawLimitSwitch = hardwareMap.get(TouchSensor.class, RobotConfig.SwitchNames.get(RobotConfig.Switches.CLAW_LIMIT));

        clawDistanceSensor = hardwareMap.get(DistanceSensor.class, RobotConfig.DistanceSensorNames.get(RobotConfig.DistanceSensors.CLAW_DISTANCE_SENSOR));

        for (RobotConfig.DriveMotors motor : RobotConfig.DriveMotors.values()) {
            driveMotors.put(motor, hardwareMap.get(DcMotor.class, RobotConfig.DriveMotorNames.get(motor)));
            Objects.requireNonNull(driveMotors.get(motor)).setDirection(RobotConfig.DriveMotorsDirections.get(motor));
            // TODO: figure out these settings
            Objects.requireNonNull(driveMotors.get(motor)).setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            Objects.requireNonNull(driveMotors.get(motor)).setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            Objects.requireNonNull(driveMotors.get(motor)).setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            Objects.requireNonNull(driveMotors.get(motor)).setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

//            Objects.requireNonNull(driveMotors.get(motor)).setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        slidesMotor1.setDirection(DcMotor.Direction.FORWARD);
        slidesMotor2.setDirection(DcMotor.Direction.REVERSE);
        secondarySlidesMotor.setDirection(DcMotor.Direction.FORWARD);

        if (desiredSlidesState == SlidesState.UNREADY) {//if the slides have yet to be initialised then reset the encoders for the slides and set the slide state to retracted
            this.telemetry.addData("desired string state", desiredSlidesState.toString());
            slidesMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            slidesMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            slidesMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            slidesMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            secondarySlidesMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            secondarySlidesMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            desiredSlidesState = SlidesState.RETRACTED;
        }
        slidesMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slidesMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        secondarySlidesMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    /** Returns the position of the robot.
     */
    public Position getPosition() {
        return positionManager.position;
    }
}


/** Maps the robot's hardware to their names in the OpMode configuration, and contains any other necessary constants
 *  pertaining to the robot's state.
 */
class RobotConfig {
    enum DistanceSensors {CLAW_DISTANCE_SENSOR}
    enum Switches {SLIDES_LIMIT, CLAW_LIMIT}
    enum Motors {SLIDES_MOTOR_1, SLIDES_MOTOR_2, SECONDARY_SLIDES_MOTOR}
    public enum DriveMotors {REAR_LEFT, REAR_RIGHT, FRONT_LEFT, FRONT_RIGHT};
    enum Servos {CLAW_ROTATOR, CLAW, CLAW_INDICATOR, CLAW_LIMIT_SWITCH_SERVO, SECONDARY_CLAW, SECONDARY_CLAW_ROTATOR}

    public static final Map<DistanceSensors, String> DistanceSensorNames = new HashMap<DistanceSensors, String>() {{
        put(DistanceSensors.CLAW_DISTANCE_SENSOR, "distance_sensor");
    }};

    public static final Map<Switches, String> SwitchNames = new HashMap<Switches, String>() {{
        put(Switches.SLIDES_LIMIT, "slides_limit");
        put(Switches.CLAW_LIMIT, "claw_limit");
    }};

    public static final Map<Motors, String> MotorNames = new HashMap<Motors, String>() {{
        put(Motors.SLIDES_MOTOR_1, "slides_motor_1");
        put(Motors.SLIDES_MOTOR_2, "slides_motor_2");
        put(Motors.SECONDARY_SLIDES_MOTOR, "secondary_slides_motor");
    }};

    public static final Map<DriveMotors, String> DriveMotorNames = new HashMap<DriveMotors, String>() {{
        put(DriveMotors.REAR_LEFT, "rear_left");
        put(DriveMotors.REAR_RIGHT, "rear_right");
        put(DriveMotors.FRONT_LEFT, "front_left");
        put(DriveMotors.FRONT_RIGHT, "front_right");
    }};

    public static final Map<DriveMotors, DcMotor.Direction> DriveMotorsDirections = new HashMap<DriveMotors, DcMotor.Direction>() {{
        put(DriveMotors.FRONT_LEFT, DcMotor.Direction.REVERSE);
        put(DriveMotors.REAR_LEFT, DcMotor.Direction.REVERSE);
        put(DriveMotors.FRONT_RIGHT, DcMotor.Direction.REVERSE);
        put(DriveMotors.REAR_RIGHT, DcMotor.Direction.REVERSE);
    }};

    public static final Map<Servos, String> ServoNames = new HashMap<Servos, String>() {{
        put(Servos.CLAW_ROTATOR, "claw_rotator");
        put(Servos.CLAW, "claw");
        put(Servos.CLAW_INDICATOR, "claw_indicator");
        put(Servos.CLAW_LIMIT_SWITCH_SERVO, "claw_limit_switch_servo");
        put(Servos.SECONDARY_CLAW, "secondary_claw");
        put(Servos.SECONDARY_CLAW_ROTATOR, "secondary_claw_rotator");
    }};
}
