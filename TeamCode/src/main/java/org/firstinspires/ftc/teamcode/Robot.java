/* Authors: Arin Khare, Kai Vernooy
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
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

    public enum SlidesState {RETRACTED, LOW, LOW_LOWERED, MEDIUM, MEDIUM_LOWERED, HIGH, HIGH_LOWERED, UNREADY}
    public enum ParkingPosition {INSIDE, MIDDLE, OUTSIDE}
    public enum HorseshoeState {FRONT, REAR}
    // public enum CompliantWheelsState {OFF, ON}

    public static SlidesState desiredSlidesState = SlidesState.UNREADY;
    public HorseshoeState desiredHorseshoeState;
    // public CompliantWheelsState desiredCompliantWheelsState;

    enum BarcodeScanState {CHECK_SCAN, SCAN}

    public BarcodeScanState barcodeScanState;
    public enum BarcodeScanResult {LEFT, CENTER, RIGHT};

    static final int MAX_BARCODE_ATTEMPTS = 40;                           // How many times to try scanning the barcode before giving up
    static final int MIN_BARCODE_REPEAT = MAX_BARCODE_ATTEMPTS / 2 + 1;

    int numBarcodeAttempts;                                               // Amount of current attempts to scan the barcode
    Map<BarcodeScanResult, Integer> barcodeScanResultMap;                 // An array representing a histogram of the scan results.
    BarcodeScanResult barcodeScanResult;                                  // Represents the final decided barcode state

    public void resetBarcodeScanMap() {
        barcodeScanResultMap = new HashMap<BarcodeScanResult, Integer>() {{
            put(BarcodeScanResult.LEFT, 0);
            put(BarcodeScanResult.CENTER, 0);
            put(BarcodeScanResult.RIGHT, 0);
        }};
    }

    enum MovementMode {NORMAL, FINE, ULTRA_FINE}
    MovementMode movementMode = MovementMode.NORMAL;
    boolean wheelSpeedAdjustment = false;

    HashMap<RobotConfig.DriveMotors, DcMotor> driveMotors = new HashMap<RobotConfig.DriveMotors, DcMotor>();

    // Hardware
    public DcMotor slidesLeft, slidesRight;
    public Servo horseshoe;
    public Servo horseshoeIndicator;

    // Other
    public Telemetry telemetry;
    public ElapsedTime elapsedTime;

    // Positioning
    public PositionManager positionManager;

    public Robot(HardwareMap hardwareMap, Telemetry telemetry, ElapsedTime elapsedTime) {
        this.telemetry = telemetry;
        this.elapsedTime = elapsedTime;
        positionManager = new PositionManager(hardwareMap, telemetry);

        numBarcodeAttempts = 0;
        resetBarcodeScanMap();

        desiredHorseshoeState = HorseshoeState.FRONT;

        slidesLeft = hardwareMap.get(DcMotor.class, RobotConfig.MotorNames.get(RobotConfig.Motors.SLIDES_LEFT));
        slidesRight = hardwareMap.get(DcMotor.class, RobotConfig.MotorNames.get(RobotConfig.Motors.SLIDES_RIGHT));
        horseshoe = hardwareMap.get(Servo.class, RobotConfig.ServoNames.get(RobotConfig.Servos.HORSESHOE));
//        clawLEDs=hardwareMap.get(DcMotor.class,"LED");
//        clawLEDs.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        clawLEDs.setDirection(DcMotorSimple.Direction.FORWARD);
        horseshoeIndicator = hardwareMap.get(Servo.class, RobotConfig.ServoNames.get(RobotConfig.Servos.HORSESHOE_INDICATOR));

        for (RobotConfig.DriveMotors motor : RobotConfig.DriveMotors.values()) {
            driveMotors.put(motor, hardwareMap.get(DcMotor.class, RobotConfig.DriveMotorNames.get(motor)));
            Objects.requireNonNull(driveMotors.get(motor)).setDirection(RobotConfig.DriveMotorsDirections.get(motor));
            Objects.requireNonNull(driveMotors.get(motor)).setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            Objects.requireNonNull(driveMotors.get(motor)).setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            Objects.requireNonNull(driveMotors.get(motor)).setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            Objects.requireNonNull(driveMotors.get(motor)).setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

//            Objects.requireNonNull(driveMotors.get(motor)).setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        slidesLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        slidesRight.setDirection(DcMotorSimple.Direction.REVERSE);

        if(desiredSlidesState ==SlidesState.UNREADY) {//if the slides have yet to be initialised then reset the encoders for the slides and set the slide state to retracted
            slidesLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            slidesRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            slidesLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            slidesRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            desiredSlidesState = SlidesState.RETRACTED;
        }
        slidesLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slidesRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
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
    enum Motors {SLIDES_LEFT, SLIDES_RIGHT}
    public enum DriveMotors {REAR_LEFT, REAR_RIGHT, FRONT_LEFT, FRONT_RIGHT};
    enum Servos {HORSESHOE, HORSESHOE_INDICATOR}

    public static final Map<Motors, String> MotorNames = new HashMap<Motors, String>() {{

        put(Motors.SLIDES_LEFT, "slides_left");
        put(Motors.SLIDES_RIGHT, "slides_right");
    }};

    public static final Map<DriveMotors, String> DriveMotorNames = new HashMap<DriveMotors, String>() {{
        put(DriveMotors.REAR_LEFT, "rear_left");
        put(DriveMotors.REAR_RIGHT, "rear_right");
        put(DriveMotors.FRONT_LEFT, "front_left");
        put(DriveMotors.FRONT_RIGHT, "front_right");
    }};

    public static final Map<DriveMotors, DcMotor.Direction> DriveMotorsDirections = new HashMap<DriveMotors, DcMotor.Direction>() {{
        put(DriveMotors.FRONT_LEFT, DcMotor.Direction.FORWARD);
        put(DriveMotors.REAR_LEFT, DcMotor.Direction.FORWARD);
        put(DriveMotors.FRONT_RIGHT, DcMotor.Direction.REVERSE);
        put(DriveMotors.REAR_RIGHT, DcMotor.Direction.REVERSE);
    }};

    public static final Map<Servos, String> ServoNames = new HashMap<Servos, String>() {{
        put(Servos.HORSESHOE, "Horseshoe");
        put(Servos.HORSESHOE_INDICATOR, "Horseshoe Indicator");
    }};
}
