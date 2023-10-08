package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Objects;


/** Stores the Robot's hardware and position.
 *  Also has a "desired state" for mechanism driving.
 */
public class Robot {
    static class Config { // access via Robot.Config
        public enum MotorConfigs { // (name, direction, behavior)
            //drive motors
            FRONT_LEFT  ("front_left", DcMotor.Direction.REVERSE, DcMotor.ZeroPowerBehavior.FLOAT),
            FRONT_RIGHT ("front_right",DcMotor.Direction.REVERSE, DcMotor.ZeroPowerBehavior.FLOAT),
            REAR_LEFT   ("rear_left",  DcMotor.Direction.REVERSE, DcMotor.ZeroPowerBehavior.FLOAT),
            REAR_RIGHT  ("rear_right", DcMotor.Direction.REVERSE, DcMotor.ZeroPowerBehavior.FLOAT),
            
            //linear slides motors
            ONE         ("slides_motor_1",         DcMotor.Direction.FORWARD, DcMotor.ZeroPowerBehavior.BRAKE),
            TWO         ("slides_motor_2",         DcMotor.Direction.REVERSE, DcMotor.ZeroPowerBehavior.BRAKE),
            SECONDARY   ("secondary_slides_motor", DcMotor.Direction.FORWARD, DcMotor.ZeroPowerBehavior.BRAKE);
            
            private String name;
            private DcMotor.Direction direction;
            private DcMotor.ZeroPowerBehavior behavior
            
            /**Creates motor configuration
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
        }

        public enum ServoConfigs { // (name, direction) // not sure if you need direction
            //not sure what to put here yet
            
            private String name;
            private Servo.Direction direction;

            /**Creates servo configuration
             * @param name the name of the component
             * @param direction the direction of the servo
             */
            private ServoConfigs(final String name, final Servo.Direction direction) {
                this.name      = name;
                this.direction = direction;
            }
            
            public String          getName()      { return name; }
            public Servo.Direction getDirection() {return direction; }
        }

        public enum SwitchConfigs { // (name)
            SLIDES_LIMIT("slides_limit");

            /**Creates switch configuration
             * @param name the name of the component
             */
            private String name;
            private SwitchConfigs(final String name) {
                this.name = name;
            }
            
            public String getName() { return name; }
        }
        /*more enums to come*/
    }

    //enum spam
    public enum SlidesState            {RETRACTED, LOW, MEDIUM, HIGH, UNREADY, MOVE_UP, MOVE_DOWN, STOPPED, FIRST_STACK_CONE, SECOND_STACK_CONE}
    public enum ParkingPosition        {INSIDE, MIDDLE, OUTSIDE}
    
    //no clue what secondary is
    public enum SecondarySlidesState   {RETRACTED, EXTENDED, PLACE_CONE, SWEEP_EXTENDED, INITIAL_EXTENDED, FINAL_RETRACTED}
    public enum SecondarySystemStatus  {ON, OFF}
    public enum SecondaryConeRetrieved {DONE, NOT_DONE}
    
    // State
    public static SlidesState     desiredSlidesState = SlidesState.UNREADY; // why is this one static?
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

    public Robot(HardwareMap hardwareMap, Telemetry telemetry, ElapsedTime elapsedTime) {
        this.telemetry   = telemetry;
        this.elapsedTime = elapsedTime;
        positionManager  = new PositionManager(hardwareMap, telemetry);
        driveMotors      = new DriveMotors(hardwareMap);
        slidesMotors     = new SlideMotors(hardwareMap);
        slidesLimitSwitch= hardwareMap.get(TouchSensor.class, Config.SwitchConfigs.SLIDES_LIMIT.getName());
        
        //probably some servos 
        
        if (desiredSlidesState == SlidesState.UNREADY) { //if the slides have yet to be initialised then reset the encoders for the slides and set the slide state to retracted
            this.telemetry.addData("desired string state", desiredSlidesState.toString());
            Motors.resetMotor(one);
            Motors.resetMotor(two);
            Motors.resetMotor(secondary);
            desiredSlidesState = SlidesState.RETRACTED;
        }
    }
    
    public Position getPosition() {
        return positionManager.position;
    }
}

abstract class Motors { // this might be a bad abstraction
    public static void resetMotor(DcMotor motor) {
        Objects.requireNonNull(motor).setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Objects.requireNonNull(motor).setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    static void setMotorToConfig(DcMotor motor, Config.MotorConfigs config) {
        Objects.requireNonNull(motor).setDirection(config.getDirection());
        Objects.requireNonNull(motor).setZeroPowerBehavior(config.getZeroPowerBehavior());
    }
}

class DriveMotors extends Motors {
    public DcMotor frontLeft, frontRight, rearLeft, rearRight;
    public DriveMotors(HardwareMap hardwareMap) {
        frontLeft = hardwareMap.get(DcMotor.class, Config.MotorConfigs.FRONT_LEFT.getName());
        frontRight= hardwareMap.get(DcMotor.class, Config.MotorConfigs.FRONT_RIGHT.getName());
        rearLeft  = hardwareMap.get(DcMotor.class, Config.MotorConfigs.REAR_LEFT.getName());
        rearRight = hardwareMap.get(DcMotor.class, Config.MotorConfigs.REAR_RIGHT.getName());
        //pass dcmotor reference and set to config
        setMotorToConfig(frontLeft, Config.MotorConfigs.FRONT_LEFT);
        setMotorToConfig(frontRight,Config.MotorConfigs.FRONT_RIGHT);
        setMotorToConfig(rearLeft,  Config.MotorConfigs.REAR_LEFT);
        setMotorToConfig(rearRight, Config.MotorConfigs.REAR_RIGHT);
    }
    @Override
    static void setMotorToConfig(DcMotor motor, Config.MotorConfigs config) {
        super.setMotorToConfig(motor, config);
        resetMotor(motor);
    }
}

class SlidesMotors extends Motors {
    public DcMotors one, two, secondary;
    public DriveMotors(HardwareMap hardwareMap) {
        one       = hardwareMap.get(DcMotor.class, Config.MotorConfigs.ONE.getName());
        two       = hardwareMap.get(DcMotor.class, Config.MotorConfigs.TWO.getName());
        secondary = hardwareMap.get(DcMotor.class, Config.MotorConfigs.SECONDARY.getName());
        
        setMotorToConfig(one, Config.Config.MotorConfigs.ONE);
        setMotorToConfig(two, Config.Config.MotorConfigs.TWO);
        setMotorToConfig(secondary, Config.Config.MotorConfigs.SECONDARY);
    }
}
