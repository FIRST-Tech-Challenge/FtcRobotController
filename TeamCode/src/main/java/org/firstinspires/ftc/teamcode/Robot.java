package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Objects;


/** Stores the Robot's hardware and position.
 *  Also has a "desired state" for mechanism driving.
 */
public class Robot {
    /**
     * Stores motor configs
     */
    public enum MotorConfigs {
        //drive motors
        FRONT_LEFT  ("front_left", DcMotor.Direction.REVERSE, DcMotor.ZeroPowerBehavior.FLOAT),
        FRONT_RIGHT ("front_right",DcMotor.Direction.REVERSE, DcMotor.ZeroPowerBehavior.FLOAT),
        REAR_LEFT   ("rear_left",  DcMotor.Direction.REVERSE, DcMotor.ZeroPowerBehavior.FLOAT),
        REAR_RIGHT  ("rear_right", DcMotor.Direction.REVERSE, DcMotor.ZeroPowerBehavior.FLOAT),
        
        //linear slides motors
        ONE         ("slides_motor_1",         DcMotor.Direction.FORWARD, DcMotor.ZeroPowerBehavior.BRAKE),
        TWO         ("slides_motor_2",         DcMotor.Direction.REVERSE, DcMotor.ZeroPowerBehavior.BRAKE),
        SECONDARY   ("secondary_slides_motor", DcMotor.Direction.FORWARD, DcMotor.ZeroPowerBehavior.BRAKE);
        
        private final String name;
        private final DcMotor.Direction direction;
        private final DcMotor.ZeroPowerBehavior behavior;
        
        /**
         * Creates motor configuration
         * @param name the name of component
         * @param direction the direction of the motor
         * @param behavior the zeroPowerBehavior of the motor
         */
        private MotorConfigs(final String name, final DcMotor.Direction direction, final DcMotor.ZeroPowerBehavior behavior) {
            this.name      = name;
            this.direction = direction;
            this.behavior  = behavior;
        }
        
        public String                    getName()              { return name; }
        public DcMotor.Direction         getDirection()         { return direction; }
        public DcMotor.ZeroPowerBehavior getZeroPowerBehavior() { return behavior; }

        /**
         * Configures and retrieves motor
         * @param config The desired motor
         * @param hardwareMap hardwareMap
         * @return Motor from config
         */
        public DcMotor initalize(Robot.MotorConfigs config, HardwareMap hardwareMap) {
            DcMotor motor = hardwareMap.get(DcMotor.class, config.getName());
            Objects.requireNonNull(motor, "Motor " + config.getName() + " not found.");
            motor.setDirection(config.getDirection());
            motor.setZeroPowerBehavior(config.getZeroPowerBehavior());
            return motor;
        }
    }

    /**
     * Stores servo configs
     */
    public enum ServoConfigs {
        //not sure what to put here yet
        
        private final String name;
        private final Servo.Direction direction;

        /**
         * Creates servo configuration
         * @param name the name of the component
         * @param direction the direction of the servo
         */
        private ServoConfigs(final String name, final Servo.Direction direction) {
            this.name      = name;
            this.direction = direction;
        }
        
        public String          getName()      { return name; }
        public Servo.Direction getDirection() { return direction; }

        /**
         * Configures and retrieves servo
         * @param config the desired servo
         * @param hardwareMap hardwareMap
         * @return the servo
         */
        public Servo initalize(Robot.ServoConfigs config, HardwareMap hardwareMap) {
            Servo servo = hardwareMap.get(Servo.class, config.getName());
            Objects.requireNonNull(servo, "Servo " + config.getName() + " not found.");
            motor.setDirection(config.getDirection());
            return servo;
        }
    }
    
    /**
     * Stores switch configs
     */
    public enum SwitchConfigs {
        SLIDES_LIMIT("slides_limit");

        /**
         * Creates switch configuration
         * @param name the name of the component
         */
        private final String name;
        private SwitchConfigs(final String name) {
            this.name = name;
        }
        
        public String getName() { return name; }

        /**
         * Retrieves switch
         * @param config the desired switch
         * @param hardwareMap hardwareMap
         * @return the switch
         */
        public TouchSensor initalize(Robot.SwitchConfigs config, HardwareMap hardwareMap) {
            TouchSensor sensor = hardwareMap.get(TouchSensor.class, config.getName());
            Objects.requireNonNull(sensor, "Switch " + config.getName() + " not found");
            return sensor;
        }
    }

    //Enums for state
    public enum SlidesState            {RETRACTED, LOW, MEDIUM, HIGH, UNREADY, MOVE_UP, MOVE_DOWN, STOPPED, FIRST_STACK_CONE, SECOND_STACK_CONE}
    public enum ParkingPosition        {INSIDE, MIDDLE, OUTSIDE}
    
    //no clue what secondary is
    public enum SecondarySlidesState   {RETRACTED, EXTENDED, PLACE_CONE, SWEEP_EXTENDED, INITIAL_EXTENDED, FINAL_RETRACTED}
    public enum SecondarySystemStatus  {ON, OFF}
    public enum SecondaryConeRetrieved {DONE, NOT_DONE}
    
    // State
    public static SlidesState     desiredSlidesState = SlidesState.UNREADY;
    public SecondarySlidesState   desiredSecondarySlidePosition;

    public SecondarySystemStatus  secondarySystemStatus;
    public SecondaryConeRetrieved secondaryConeRetrieved;

    public boolean                previousSlidesLimitSwitchState = false;
    public SecondarySlidesState   previousDesiredSecondarySlidePosition;
    public SecondarySystemStatus  previousSecondarySystemStatus;

    enum MovementMode {NORMAL, FINE, ULTRA_FINE}
    MovementMode movementMode = MovementMode.NORMAL;
    boolean wheelSpeedAdjustment = false;
    
    // Hardware
    public DriveMotors driveMotors;
    public SlidesMotors slidesMotors;

    public TouchSensor slidesLimitSwitch;
    //servos
    //idk what else

    // Other
    public Telemetry telemetry;
    public ElapsedTime elapsedTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    public PositionManager positionManager;

    /** Creates robot and gets components
     * @param hardwareMap map components to hardware components
     * @param telemetry dunno
     * @param elapsedTime dunno
     */
    public Robot(HardwareMap hardwareMap, Telemetry telemetry, ElapsedTime elapsedTime) {
        this.telemetry   = telemetry;
        this.elapsedTime = elapsedTime;
        positionManager  = new PositionManager(hardwareMap, telemetry);
        driveMotors      = new DriveMotors(hardwareMap);
        slidesMotors     = new SlidesMotors(hardwareMap);
        slidesLimitSwitch= Robot.SwitchConfigs.initalize(Robot.SwitchConfigs.SLIDES_LIMIT, hardwareMap);
        
        //probably some servos 
        
        if (desiredSlidesState == SlidesState.UNREADY) { //if the slides have yet to be initialised then reset the encoders for the slides and set the slide state to retracted
            this.telemetry.addData("desired string state", desiredSlidesState.toString());
            Motors.resetEncoder(one);
            Motors.resetEncoder(two);
            Motors.resetEncoder(secondary);
            desiredSlidesState = SlidesState.RETRACTED;
        }
    }
    
    public Position getPosition() {
        return positionManager.position;
    }
}

/**
 * Container template for motors
 */
abstract class Motors { // this might be a bad abstraction
    /**
     * Resets a motor's encoder
     * @param motor the motor acted upon
     */
    public static void resetEncoder(DcMotor motor) {
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}

/**
 * stores drive motors
 */
class DriveMotors extends Motors {
    public DcMotor frontLeft, frontRight, rearLeft, rearRight;
    /**
     * Takes hardwareMap and gets drive motors from it
     * @param hardwareMap the hardwareMap used
     */
    public DriveMotors(HardwareMap hardwareMap) {
        frontLeft = Robot.MotorConfigs.initalize(Robot.MotorConfigs.FRONT_LEFT, hardwareMap);
        frontRight= Robot.MotorConfigs.initalize(Robot.MotorConfigs.FRONT_RIGHT, hardwareMap);
        rearLeft  = Robot.MotorConfigs.initalize(Robot.MotorConfigs.REAR_LEFT, hardwareMap);
        rearRight = Robot.MotorConfigs.initalize(Robot.MotorConfigs.REAR_RIGHT, hardwareMap);

        resetEncoder(frontLeft);
        resetEncoder(frontRight);
        resetEncoder(rearLeft);
        resetEncoder(rearRight);
    }
}
/**
 * stores slides motors
 */
class SlidesMotors extends Motors {
    public DcMotor one, two, secondary;
    /**
     * Gets slides motors from hardware
     * @param hardwareMap the hardwareMap used
     */
    public SlidesMotors(HardwareMap hardwareMap) {
        one       = Robot.MotorConfigs.initalize(Robot.MotorConfigs.ONE, hardwareMap);
        two       = Robot.MotorConfigs.initalize(Robot.MotorConfigs.TWO, hardwareMap);
        secondary = Robot.MotorConfigs.initalize(Robot.MotorConfigs.SECONDARY, hardwareMap);
    }
}
