package org.firstinspires.ftc.teamcode.Robots;

import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import java.util.ArrayList;
import java.util.Arrays;

public interface Robot202 {
    //Game Variables
    double robot_width = 15.0;
    double robot_length = 15.0;

    //Robot Hardware

    ArrayList<String> dc_motor_names = new ArrayList<>(Arrays.asList("joint1right", "joint2"));
    double[] max_power = {0.1256486834658387658, 0.4};
    double[] min_power = {-0.5, -0.4};
    double[] motor_max_positions = {Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY};
    double[] motor_min_positions = {Double.NEGATIVE_INFINITY, Double.NEGATIVE_INFINITY};
    boolean[] invert_dc_motors = {false, false};
    double[] p_weights = {0.02, 0.02};

    ArrayList<String> servo_names = new ArrayList<>(Arrays.asList("clawAligner", "claw"));
    double[] servo_max_positions = {1.0, 1.0};
    double[] servo_min_positions = {0, 0};

    ArrayList<String> cr_servo_names = new ArrayList<String>(Arrays.asList());
    boolean[] invert_cr_servos = {};

    ArrayList<String> distance_sensor_names = new ArrayList<>(Arrays.asList());

    ArrayList<String> touch_sensor_names = new ArrayList<>(Arrays.asList());

    ArrayList<String> color_sensor_names = new ArrayList<>(Arrays.asList());

    ArrayList<String> led_names = new ArrayList<>(Arrays.asList());

    //Driving
    double strafe = 0.0;
    double turning_weight = 0.0;
    double distance_weight = 0.0;
    double distance_weight_two = 0.0;

    boolean locked_motion = true;
    boolean locked_rotation = false;

    //PID
    boolean usePID = true;
    double p_weight = 0.025; // 0.025
    double d_weight = 0; // 0.085

    AxesOrder axesOrder = AxesOrder.YXZ;
    boolean invertIMU = false;

    //Road Runner
    boolean useRoadRunner = false;
    double ticks_per_revolution = 0.0;
    double wheel_radius = 0.0;
    double gear_ratio = 0.0;
    double lateral_distance = 0.0;
    double forward_offset = 4.0;
    boolean integer_overflow = false;
    boolean[] invert_encoders = {false, false, false};

    //Road Runner Tuning
    double forward_multiplier = 1.0;
    double strafing_multiplier = 1.0;
    double turning_multiplier = 1.0;

    //Tensorflow
    String VUFORIA_KEY =
            "Vuforia Key";
    String webcam_name = "Webcam 1";
    double camera_zoom = 1.0;
    String TFOD_MODEL_ASSET = "PowerPlay.tflite";
    String TFOD_MODEL_FILE = "/sdcard/FIRST/tflitemodels/CustomTeamModel.tflite";
    Float min_confidence = 0.75f;
    int input_size = 300;
    boolean useAsset = true;
    String[] LABELS = {
            "1 Bolt",
            "2 Bulb",
            "3 Panel"
    };

    ArrayList<String> encoderNames = new ArrayList<>(Arrays.asList("leftEncoder", "rightEncoder", "frontEncoder"));

    ArrayList<String> wheel_names = new ArrayList<>(Arrays.asList("rightFront", "rightBack", "leftBack", "leftFront"));

    ArrayList<String> keys = new ArrayList<>(Arrays.asList(
            "operator a", "operator b", "operator x", "operator y", "operator dpad_up", "operator dpad_down",
            "operator dpad_left", "operator dpad_right", "operator left_bumper", "operator right_bumper",
            "driver a", "driver b", "driver x", "driver y", "driver dpad_up", "driver dpad_down",
            "driver dpad_left", "driver dpad_right", "driver left_bumper", "driver right_bumper",
            "operator left_stick_x", "operator right_stick_x", "operator left_stick_y", "operator right_stick_y",
            "operator left_trigger", "operator right_trigger", "driver left_trigger"
    ));
}