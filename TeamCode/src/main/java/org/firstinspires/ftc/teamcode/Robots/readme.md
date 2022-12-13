## Instructions

Hey hey hey!

If you are coding for multiple robots, then include the code in here. Then, you can extend the 
interface for the code inside of teamcode/Robot.java

You will also want to make different DriverControlled and Autonomous code for the 2 teams

Copy this and paste it into a new file:

package org.firstinspires.ftc.teamcode.Robots;

import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import java.util.ArrayList;
import java.util.Arrays;

public interface {Interface Name} {
    //Game Variables
    //ex. motor positions, servo positions, positions on the field

    double[] motorPositions = {0, 100, 1000}; //Even though motor positions are ints, they have to be doubles
    ArrayList<String> armPositionNames = new ArrayList<>(Arrays.asList("Zero", "One Hundred", "One Thousand"));

    double[] servoPositions = {0.0, 1.0};
    ArrayList<String> servoPositionNames = new ArrayList<>(Arrays.asList("Default", "Fully Rotated"));

    double[][] area = {{0, 0}, {20, 0}, {20, 20}, {0, 20}};
    double[] point = {30, 40};

    double robot_width = 15.0; //width = distance along strafing axis
    double robot_length = 15.0; //length = distance along driving axis

    //Robot Hardware

    ArrayList<String> dc_motor_names = new ArrayList<>(Arrays.asList()); //Pretty simple, this is just the names of the dc motors
                                                                                //that are connected to the control hubs, excluding
                                                                                //the four drivetrain motors. 
    double[] max_power = {};
    double[] min_power = {};
    double[] motor_max_positions = {}; //just keep as 1 if encoders disabled, Double.POSITIVE_INFINITY if we don't want a limit
    double[] motor_min_positions = {}; //just keep as -1 if encoders disabled, Double.NEGATIVE_INFINITY if we don't want a limit
    int[] dc_motor_directions = {}; //0 for forward, 1 for reverse
    double[] p_weights = {}; //for PID; multiplied by difference (encoder ticks); I recommend 0.05

    ArrayList<String> servo_names = new ArrayList<>(Arrays.asList());
    double[] servo_max_positions = {};
    double[] servo_min_positions = {};

    ArrayList<String> distance_sensor_names = new ArrayList<>(Arrays.asList());

    ArrayList<String> touch_sensor_names = new ArrayList<>(Arrays.asList());

    ArrayList<String> color_sensor_names = new ArrayList<>(Arrays.asList());

    ArrayList<String> led_names = new ArrayList<>(Arrays.asList());

    //Driving
    double strafe = 1.0; //ratio of strafe speed to forward speed. Ex. if strafe speed was 3 ft/s and forward speed was 4 ft/s, this variable is 0.75
    double turning_weight = 1.0;
    double distance_weight = 1.0;
    double distance_weight_two = 1.0; //multiplied by inches, only for RoadRunner

    boolean locked_motion = false; //Motion is relative to player, not to robot
    boolean locked_rotation = false; //Rotation is relative to player, not to robot
    //These must be false if not using PID or RoadRunner

    //PID
    boolean usePID = true;
    double p_weight = 0; //multiplied by difference (radians); 0.025
    double d_weight = 0; //multiplied by derivative of difference (radians per millisecond); 0.085

    AxesOrder axesOrder = AxesOrder.ZYX; //change this depending on the IMU orientation
    boolean invertIMU = false;

    //Road Runner
    boolean useRoadRunner = false; //can only use ONE of RoadRunner and PID
    double ticks_per_revolution = 0.0; //encoder ticks per revolution for dead wheels
    double wheel_radius = 0.0; //in inches
    double gear_ratio = 0.0; //output (wheel) speed / input (encoder) speed
    double lateral_distance = 0.0;  //inches; distance between the left and right dead wheels
    double forward_offset = 4.0; //inches; offset of the lateral wheel
    boolean integer_overflow = false;
    int[] encoder_directions = {1, 1, 1}; //0 for forward, 1 for reverse

    //Road Runner Tuning
    double forward_multiplier = 1.0;
    double strafing_multiplier = 1.0;
    double turning_multiplier = 1.0;
    //tune these in roadrunner calibration
    //strafing/forward: if we go x inches but thought we were going 90, multiplier = x / 90
    //turning: if we turn x degrees but thought we were rotating 1800, multiplier = x / 1800 - roughly
    //kinda guess with these 2; but if we don't go/turn as far, decrease the multiplier, and vice versa
    //more tests and more distance travelled --> more accurate.

    //Tensorflow
    String VUFORIA_KEY =
            "Vuforia Key"; //actually have to get ur own key lol
    String webcam_name = "Webcam 1";
    double camera_zoom = 1.0; //use greater zoom for if the distance is > 50 cm; must be >= 1
    String TFOD_MODEL_ASSET = "PowerPlay.tflite";
    // Use loadModelFromAsset() if the TF Model is built in as an asset by Android Studio
    String TFOD_MODEL_FILE = "/sdcard/FIRST/tflitemodels/CustomTeamModel.tflite";
    // Use loadModelFromFile() if you have downloaded a custom team model to the Robot Controller's FLASH.
    Float min_confidence = 0.75f;
    int input_size = 300;
    boolean useAsset = true;
    String[] LABELS = {
            "1 Bolt",
            "2 Bulb",
            "3 Panel"
    };

    //No need to ever change this

    ArrayList<String> encoderNames = new ArrayList<>(Arrays.asList("leftEncoder", "rightEncoder", "frontEncoder")); //Dead Wheels for RoadRunner

    ArrayList<String> wheel_names = new ArrayList<>(Arrays.asList("rightFront", "rightBack", "leftBack", "leftFront"));
    //wheel order is front right, back right, back left, front left

    ArrayList<String> keys = new ArrayList<>(Arrays.asList(
            "operator a", "operator b", "operator x", "operator y", "operator dpad_up", "operator dpad_down",
            "operator dpad_left", "operator dpad_right", "operator left_bumper", "operator right_bumper",
            "driver a", "driver b", "driver x", "driver y", "driver dpad_up", "driver dpad_down",
            "driver dpad_left", "driver dpad_right", "driver left_bumper", "driver right_bumper",
            "operator left_stick_x", "operator right_stick_x", "operator left_stick_y", "operator right_stick_y",
            "operator left_trigger", "operator right_trigger", "driver left_trigger"
    ));

}