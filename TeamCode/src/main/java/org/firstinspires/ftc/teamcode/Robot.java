package org.firstinspires.ftc.teamcode;

import android.media.Image;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;

/** Stores the Robot's hardware and position.
 *  Also has a "desired state" for mechanism driving.
 */
public class Robot {
    /**
     * Stores motor configs
     */
    public enum MotorConfigs {
        //drive motors
        FRONT_LEFT  ("front_left", DcMotor.Direction.FORWARD, DcMotor.ZeroPowerBehavior.FLOAT),
        FRONT_RIGHT ("front_right",DcMotor.Direction.FORWARD, DcMotor.ZeroPowerBehavior.FLOAT),
        REAR_LEFT   ("rear_left",  DcMotor.Direction.FORWARD, DcMotor.ZeroPowerBehavior.FLOAT),
        REAR_RIGHT  ("rear_right", DcMotor.Direction.FORWARD, DcMotor.ZeroPowerBehavior.FLOAT),
        
        //linear slides motors
        SLIDES         ("slides_motor",         DcMotor.Direction.FORWARD, DcMotor.ZeroPowerBehavior.BRAKE),

        COMPLIANT_MOTOR_LEFT ("compliant_motor_left", DcMotor.Direction.FORWARD, DcMotor.ZeroPowerBehavior.FLOAT),
        COMPLIANT_MOTOR_RIGHT ("compliant_motor_right", DcMotor.Direction.FORWARD, DcMotor.ZeroPowerBehavior.FLOAT);

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
        public static DcMotor initialize(Robot.MotorConfigs config, HardwareMap hardwareMap) {
            DcMotor motor = hardwareMap.get(DcMotor.class, config.getName());
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

        PLANE_SPRING("plane_spring", Servo.Direction.FORWARD),
        COMPARTMENT_LEFT("compartment_left", Servo.Direction.FORWARD),
        COMPARTMENT_RIGHT("compartment_right", Servo.Direction.FORWARD);
        final String name;
        final Servo.Direction direction;

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
        public static Servo initialize(Robot.ServoConfigs config, HardwareMap hardwareMap) {
            Servo servo = hardwareMap.get(Servo.class, config.getName());
            servo.setDirection(config.getDirection());
            return servo;
        }
    }
    
    /**
     * Stores switch configs
     */
//    public enum SwitchConfigs {
//        SLIDES_LIMIT("slides_limit");
//
//        /**
//         * Creates switch configuration
//         * @param name the name of the component
//         */
//        private final String name;
//        private SwitchConfigs(final String name) {
//            this.name = name;
//        }
//
//        public String getName() { return name; }
//
//        /**
//         * Retrieves switch
//         * @param config the desired switch
//         * @param hardwareMap hardwareMap
//         * @return the switch
//         */
//        public static TouchSensor initialize(Robot.SwitchConfigs config, HardwareMap hardwareMap) {
//            TouchSensor sensor = hardwareMap.get(TouchSensor.class, config.getName());
//            return sensor;
//        }
//    }

    //Enums for state
    public enum SlidesState            {RETRACTED, LOW, MEDIUM, HIGH, UNREADY, MOVE_UP, MOVE_DOWN, STOPPED};
    public enum ParkingPosition        {INSIDE, MIDDLE, OUTSIDE};

    // State
    public SlidesState     desiredSlidesState = SlidesState.UNREADY;
    public enum CompartmentState          {OPEN, CLOSED};
    public CompartmentState desiredCompartmentLeftState = CompartmentState.CLOSED;
    public CompartmentState desiredCompartmentRightState = CompartmentState.CLOSED;
    public enum CompliantWheelsState          {ON, OFF};
    public CompliantWheelsState desiredCompliantWheelsState = CompliantWheelsState.OFF;
    public enum PlaneSpringState          {UNRELEASED, RELEASED};
    public PlaneSpringState desiredPlaneStringState = PlaneSpringState.UNRELEASED;
    enum MovementMode {NORMAL, FINE, ULTRA_FINE}
    MovementMode movementMode = MovementMode.NORMAL;
    boolean wheelSpeedAdjustment = false;
    
    // Hardware
    public DcMotor slides, compliantWheelLeft, compliantWheelRight;
    public DcMotor frontLeft, frontRight, rearLeft, rearRight;
    public Servo planeSpring, compartmentLeft, compartmentRight;

//    public TouchSensor slidesLimitSwitch;
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
        frontLeft = Robot.MotorConfigs.initialize(Robot.MotorConfigs.FRONT_LEFT, hardwareMap);
        frontRight= Robot.MotorConfigs.initialize(Robot.MotorConfigs.FRONT_RIGHT, hardwareMap);
        rearLeft  = Robot.MotorConfigs.initialize(Robot.MotorConfigs.REAR_LEFT, hardwareMap);
        rearRight = Robot.MotorConfigs.initialize(Robot.MotorConfigs.REAR_RIGHT, hardwareMap);

        resetEncoder(frontLeft);
        resetEncoder(frontRight);
        resetEncoder(rearLeft);
        resetEncoder(rearRight);

        slides            = Robot.MotorConfigs.initialize(Robot.MotorConfigs.SLIDES, hardwareMap);
        compliantWheelLeft            = Robot.MotorConfigs.initialize(MotorConfigs.COMPLIANT_MOTOR_LEFT, hardwareMap);
        compliantWheelRight            = Robot.MotorConfigs.initialize(MotorConfigs.COMPLIANT_MOTOR_RIGHT, hardwareMap);
        planeSpring     = Robot.ServoConfigs.initialize(ServoConfigs.PLANE_SPRING, hardwareMap);
        compartmentLeft     = Robot.ServoConfigs.initialize(ServoConfigs.COMPARTMENT_LEFT, hardwareMap);
        compartmentRight     = Robot.ServoConfigs.initialize(ServoConfigs.COMPARTMENT_RIGHT, hardwareMap);
//        slidesLimitSwitch= Robot.SwitchConfigs.initialize(Robot.SwitchConfigs.SLIDES_LIMIT, hardwareMap);

        // Set slides state to Retracted
        if (desiredSlidesState == SlidesState.UNREADY) { //if the slides have yet to be initialised then reset the encoders for the slides and set the slide state to retracted
            this.telemetry.addData("desired string state", desiredSlidesState.toString());
            resetEncoder(slides);
            desiredSlidesState = SlidesState.RETRACTED;
        }
    }

    public static void resetEncoder(DcMotor motor) {
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    
    public Position getPosition() {
        return positionManager.position;
    }
}