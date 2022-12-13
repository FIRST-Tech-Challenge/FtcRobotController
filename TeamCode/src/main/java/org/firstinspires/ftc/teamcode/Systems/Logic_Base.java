package org.firstinspires.ftc.teamcode.Systems;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Systems.RoadRunner.StandardTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.Robot;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;

public class Logic_Base implements Robot {

    public RobotHardware robot;
    public StandardTrackingWheelLocalizer position_tracker;

    public HashMap<String, ArrayList<Object>> keybinds = new HashMap<>();
    public String[] button_types = new String[27];

    public double[] times_started = new double[dc_motor_names.size() + servo_names.size() + cr_servo_names.size()]; //in seconds
    public double[] target_positions = new double[dc_motor_names.size() + servo_names.size()];
    public double[] starting_positions = new double[dc_motor_names.size() + servo_names.size()]; //never use for dc_motors

    public int[] key_values = new int[27]; //number of times button/axis is "activated"
    public boolean[] buttons = new boolean[20]; //value of button (True or False)
    public double[] axes = new double[7]; //value of axis (1 for buttons/cycles, -1.0 to 1.0 for everything else)

    public double current_x = 0;
    public double current_y = 0;
    public double current_angle = 0;

    public double target_x = 0;
    public double target_y = 0;
    public double target_angle = 0;

    public double zero_angle = 0;

    public double current_error;
    public double previous_error;

    public long current_time;
    public long previous_time;

    public Logic_Base(RobotHardware r) {
        robot = r;
        
        for (String servo : servo_names) {
            keybinds.put(servo, new ArrayList<>());
        }
        for (String motor : dc_motor_names) {
            keybinds.put(motor, new ArrayList<>());
        }
        for (String cr_servo : cr_servo_names) {
            keybinds.put(cr_servo, new ArrayList<>());
        }
        keybinds.put("goto", new ArrayList<>());

        if ((useRoadRunner) && (usePID)) {
            throw new IllegalArgumentException("You cannot use both RoadRunner and the build-in PID");
        }
        if (((locked_motion) || (locked_rotation)) && !((useRoadRunner) || (usePID))) {
            throw new IllegalArgumentException("You can't use locked motion or rotion without a PID method");
        }
    }

    public void execute_controllers(Gamepad gamepad1, Gamepad gamepad2) {
        drive(gamepad1);
        update_buttons(gamepad1, gamepad2);
        update_robot();
    }

    public void update_button(boolean button_pressed, String button_name) {
        boolean button_active;
        int temp = keys.indexOf(button_name);
        if (("toggle").equals(button_types[temp])) {
            key_values[temp] += (button_pressed == (key_values[temp] % 2 == 0)) ? 1 : 0;
            button_active = (key_values[temp] % 4 != 0);
        } else if (("default").equals(button_types[temp])) {
            button_active = button_pressed;
        } else {
            button_active = (key_values[temp] % 2 == 0) && (button_pressed);
            key_values[temp] += (button_pressed == (key_values[temp] % 2 == 0)) ? 1 : 0;
        }
        buttons[temp] = button_active;
    }

    public void update_axis(double axis, String axis_name) {
        double axis_value;
        int temp = keys.indexOf(axis_name);
        if (("toggle").equals(button_types[temp])) {
            key_values[temp] += ((Math.abs(axis) > 0.1) == (key_values[temp] % 2 == 0)) ? 1 : 0;
            axis_value = key_values[temp] % 4 != 0 ? 1 : 0;
        } else if (("default").equals(button_types[temp])) {
            axis_value = axis;
        } else {
            axis_value = (key_values[temp] % 2 == 0) && (Math.abs(axis) > 0.1) ? 1 : 0;
            key_values[temp] += ((Math.abs(axis) > 0.1) == (key_values[temp] % 2 == 0)) ? 1 : 0;
        }
        axes[temp - 20] = axis_value;
    }

    public void update_buttons(Gamepad gamepad1, Gamepad gamepad2) {
        update_button(gamepad1.a, "driver a");
        update_button(gamepad1.b, "driver b");
        update_button(gamepad1.x, "driver x");
        update_button(gamepad1.y, "driver y");
        update_button(gamepad1.dpad_up, "driver dpad_up");
        update_button(gamepad1.dpad_down, "driver dpad_down");
        update_button(gamepad1.dpad_left, "driver dpad_left");
        update_button(gamepad1.dpad_right, "driver dpad_right");
        update_button(gamepad1.left_bumper, "driver left_bumper");
        update_button(gamepad1.right_bumper, "driver right_bumper");
        update_axis(gamepad1.left_trigger, "driver left_trigger");

        update_button(gamepad2.a, "operator a");
        update_button(gamepad2.b, "operator b");
        update_button(gamepad2.x, "operator x");
        update_button(gamepad2.y, "operator y");
        update_button(gamepad2.dpad_up, "operator dpad_up");
        update_button(gamepad2.dpad_down, "operator dpad_down");
        update_button(gamepad2.dpad_left, "operator dpad_left");
        update_button(gamepad2.dpad_right, "operator dpad_right");
        update_button(gamepad2.left_bumper, "operator left_bumper");
        update_button(gamepad2.right_bumper, "operator right_bumper");
        update_axis(gamepad2.left_stick_x, "operator left_stick_x");
        update_axis(0 - gamepad2.left_stick_y, "operator left_stick_y"); //negative because down means positive for FTC controllers
        update_axis(gamepad2.right_stick_x, "operator right_stick_x");
        update_axis(0 - gamepad2.right_stick_y, "operator right_stick_y");
        update_axis(gamepad2.left_trigger, "operator left_trigger");
        update_axis(gamepad2.right_trigger, "operator right_trigger");
    }

    public void update_robot() {
        for (Map.Entry<String, ArrayList<Object>> element : keybinds.entrySet()) { //for every element in keybinds

            ArrayList<Object> object_keys = element.getValue(); //object_keys = what the motor maps to

            int number_of_keys = object_keys.size() / 4; //number of keys that map to the motor
            boolean object_is_active = false; //object is active iff at least one key that maps to it is activated

            boolean isDcMotor;
            int key_index;

            for (int i = 0; i < number_of_keys; i++) {
                key_index = keys.indexOf((String) object_keys.get(4 * i));
                object_is_active = ((object_is_active) || ((key_index < 20) && (buttons[key_index])) || ((key_index > 19) && (Math.abs(axes[key_index - 20]) > 0.1)));
            }

            if ((dc_motor_names.contains(element.getKey())) || (servo_names.contains(element.getKey()))) {

                isDcMotor = dc_motor_names.contains(element.getKey());

                int general_list_index;
                int specific_list_index;

                if (isDcMotor) {
                    general_list_index = dc_motor_names.indexOf(element.getKey());
                    specific_list_index = dc_motor_names.indexOf(element.getKey());
                } else {
                    general_list_index = servo_names.indexOf(element.getKey()) + dc_motor_names.size();
                    specific_list_index = servo_names.indexOf(element.getKey());
                }

                if (!object_is_active) { //if we aren't pressing any relevant buttons

                    times_started[general_list_index] = -10.0;

                    if (isDcMotor) {
                        robot.dc_motor_list[specific_list_index].setPower(Math.max(min_power[specific_list_index], Math.min(max_power[specific_list_index], (target_positions[general_list_index] - robot.dc_motor_list[specific_list_index].getCurrentPosition()) * p_weights[specific_list_index])));
                    } else {
                        robot.servo_list[specific_list_index].setPosition(target_positions[general_list_index]);
                        starting_positions[general_list_index] = target_positions[general_list_index];
                    }
                    
                } else {

                    if (times_started[general_list_index] < 0) //if we're on and it's reset, un-reset it
                        times_started[general_list_index] = (double) System.nanoTime() / 1000000000.0;

                    for (int i = 0; i < number_of_keys; i++) {

                        key_index = keys.indexOf((String) object_keys.get(4 * i)); //where button is in list of keys; < 20 -> button, >= 20 -> axis
                        String type = (String) object_keys.get(4 * i + 1);

                        if ((key_index < 20 && buttons[key_index]) || (key_index > 19 && Math.abs(axes[key_index - 20]) > 0.1)) {
                            if ((type.equals("button")) || (type.equals("cycle"))) {

                                int delta = (int) object_keys.get(4 * i + 2);
                                double[] positions = (double[]) object_keys.get(4 * i + 3);

                                if (positions.length == 1) {
                                    target_positions[general_list_index] = positions[0];
                                } else {
                                    boolean increasing = (positions[1] > positions[0]);
                                    int current_index = 0;
                                    while ((current_index < positions.length) && ((positions[current_index] < target_positions[general_list_index]) || (!increasing)) && ((positions[current_index] > target_positions[general_list_index]) || (increasing))) {
                                        current_index += 1;
                                    }
                                    if ((delta > 0) && ((current_index + 1 > positions.length)  || (positions[current_index] != target_positions[key_index]))) {
                                        current_index -= 1;
                                    }
                                    if (type.equals("cycle")) {
                                        if ((current_index + 2 > positions.length) && (delta > 0)) {
                                            current_index = 0;
                                        } else if ((current_index < 1) && (delta < 0)) {
                                            current_index = positions.length - 1;
                                        } else {
                                            current_index = Math.max(0, Math.min(current_index + delta, positions.length - 1));
                                        }
                                    } else {
                                        current_index = Math.max(0, Math.min(current_index + delta, positions.length - 1));
                                    }
                                    target_positions[general_list_index] = positions[current_index]; //change the target position
                                }
                            } else {
                                if (isDcMotor) {
                                    target_positions[general_list_index] = Math.max(Math.min(robot.dc_motor_list[specific_list_index].getCurrentPosition(),
                                        motor_max_positions[specific_list_index]), motor_min_positions[specific_list_index]);

                                    double calculated_power;

                                    if ((type.equals("toggle")) || (key_index < 20)) {
                                        calculated_power = ((double) object_keys.get(4 * i + 3)) * (((String) object_keys.get(4 * i + 2)).equals("normal") ? 1.0 : Math.min(1, ((double) System.nanoTime() / 1000000000.0 - times_started[general_list_index]) / 0.75));
                                    } else {
                                        calculated_power = axes[key_index - 20] * ( //similar to button defaults, except no gradient option
                                                (key_index > 23) ? (double) object_keys.get(4 * i + 2) : //if it's a trigger, then set it to the first val
                                                (axes[key_index - 20] < 0 ? (double) object_keys.get(4 * i + 2) : (double) object_keys.get(4 * i + 3))
                                        );
                                    }

                                    calculated_power = Math.max(min_power[specific_list_index], Math.min(max_power[specific_list_index], calculated_power));
                                    if ((robot.dc_motor_list[specific_list_index].getCurrentPosition() > motor_max_positions[specific_list_index]) && (calculated_power > 0)) {
                                        calculated_power = Math.max(min_power[specific_list_index], (motor_max_positions[specific_list_index] - robot.dc_motor_list[specific_list_index].getCurrentPosition()) * p_weights[specific_list_index]);
                                    } else if (robot.dc_motor_list[specific_list_index].getCurrentPosition() < motor_min_positions[specific_list_index] && (calculated_power < 0)) {
                                        calculated_power = Math.min((motor_min_positions[specific_list_index] - robot.dc_motor_list[specific_list_index].getCurrentPosition()) * p_weights[specific_list_index], max_power[specific_list_index]);
                                    }
                                    robot.dc_motor_list[specific_list_index].setPower(calculated_power);
                                } else {
                                    if ((type.equals("toggle")) || (key_index < 20)) {
                                        target_positions[general_list_index] = starting_positions[general_list_index] + (double) object_keys.get(4 * i + 2) * ((double) System.nanoTime() / 1000000000.0 - times_started[general_list_index]);
                                    } else {
                                        target_positions[general_list_index] += //the expression below is seconds/tick, basically; current pos + seconds/tick * depth * angles/second
                                            ((double) System.nanoTime() / 1000000000.0 - times_started[general_list_index]) * axes[key_index - 20] * (
                                                (key_index > 23) ? (double) object_keys.get(4 * i + 2) : //if it's a trigger, then set it to the first val
                                                (axes[key_index - 20] < 0 ? (double) object_keys.get(4 * i + 2) : (double) object_keys.get(4 * i + 3))
                                            );
                                        times_started[general_list_index] = (double) System.nanoTime() / 1000000000.0;
                                    }
                                    target_positions[general_list_index] = Math.max(servo_min_positions[specific_list_index], Math.min(servo_max_positions[specific_list_index], target_positions[general_list_index]));
                                    robot.servo_list[specific_list_index].setPosition(target_positions[general_list_index]);
                                }
                            }
                        }
                    }
                }
            } else if (cr_servo_names.contains(element.getKey())) {
                int index = cr_servo_names.indexOf(element.getKey());
                if (!object_is_active) {
                    robot.cr_servo_list[index].setPower(0);
                    times_started[target_positions.length + index] = -10;
                } else {
                    if (times_started[target_positions.length + index] < 0) {
                        times_started[target_positions.length + index] = (double) System.nanoTime() / 1000000000.0;
                    }
                    for (int i = 0; i < number_of_keys; i++) {

                        key_index = keys.indexOf((String) object_keys.get(4 * i)); //where button is in list of keys; < 20 -> button, >= 20 -> axis
                        String type = (String) object_keys.get(4 * i + 1);

                        if ((key_index < 20 && buttons[key_index]) || (key_index > 19 && Math.abs(axes[key_index - 20]) > 0.1)) {

                            if ((type.equals("toggle")) || (key_index < 20)) {
                                robot.cr_servo_list[index].setPower(((double) object_keys.get(4 * i + 3)) * (((String) object_keys.get(4 * i + 2)).equals("normal") ? 1.0 : Math.min(1, ((double) System.nanoTime() / 1000000000.0 - times_started[target_positions.length + index]) / 0.75)));
                            } else {
                                robot.cr_servo_list[index].setPower(axes[key_index - 20] * ( //similar to button defaults, except no gradient option
                                    (key_index > 23) ? (double) object_keys.get(4 * i + 2) : //if it's a trigger, then set it to the first val
                                    (axes[key_index - 20] < 0 ? (double) object_keys.get(4 * i + 2) : (double) object_keys.get(4 * i + 3))
                                ));
                            }
                        }
                    }
                }
            } else {
                for (int i = 0; i < number_of_keys; i++) {

                    key_index = keys.indexOf((String) object_keys.get(4 * i)); //where button is in list of keys; < 20 -> button, >= 20 -> axis

                    if ((key_index < 20 && buttons[key_index]) || (key_index > 19 && Math.abs(axes[key_index - 20]) > 0.1)) {
                        target_x = (double) object_keys.get(4 * i + 1);
                        target_y = (double) object_keys.get(4 * i + 2);
                        try {
                            target_angle = (double) object_keys.get(4 * i + 3);
                            target_angle = Math.toRadians(target_angle);
                        } catch (ClassCastException e) { //if it's a string, we do NOT reset the target angle
                            target_angle = current_angle;
                        }
                    }
                }
            }
        }
    }

    //Driving

    public void drive(Gamepad gamepad) {
        double speedFactor = 1 + 2 * gamepad.right_trigger;

        double left_stick_magnitude = Math.sqrt(gamepad.left_stick_x * gamepad.left_stick_x + gamepad.left_stick_y * gamepad.left_stick_y);
        if (left_stick_magnitude <= 0.333) left_stick_magnitude = 0.0;
        double left_stick_angle =
                (left_stick_magnitude <= 0.333) ? -Math.PI / 2.0 :
                (gamepad.left_stick_x > 0) ? Math.atan(gamepad.left_stick_y/gamepad.left_stick_x) :
                (gamepad.left_stick_x < 0) ? Math.PI + Math.atan(gamepad.left_stick_y/gamepad.left_stick_x) :
                (gamepad.left_stick_y > 0) ? Math.PI / 2.0 : -Math.PI / 2.0;
        left_stick_angle += Math.PI/2.0;
        
        double right_stick_magnitude = Math.sqrt(gamepad.right_stick_x * gamepad.right_stick_x + gamepad.right_stick_y * gamepad.right_stick_y);
        if (right_stick_magnitude <= 0.333) right_stick_magnitude = 0.0;
        double right_stick_angle =
                (right_stick_magnitude <= 0.333) ? -Math.PI / 2.0 :
                (gamepad.right_stick_x > 0) ? Math.atan(gamepad.right_stick_y/gamepad.right_stick_x) :
                (gamepad.right_stick_x < 0) ? Math.PI + Math.atan(gamepad.right_stick_y/gamepad.right_stick_x) :
                (gamepad.right_stick_y > 0) ? Math.PI / 2.0 : -Math.PI / 2.0;
        right_stick_angle += Math.PI/2.0;

        left_stick_angle = modifiedAngle(left_stick_angle);
        right_stick_angle = modifiedAngle(right_stick_angle);

        //Positive angles --> clockwise
        //Zero --> vertical

        if (usePID) {
            current_angle = 0 - robot.getAngle() - zero_angle;
        } else if (useRoadRunner) {
            position_tracker.update();
            Pose2d currentPose = position_tracker.getPoseEstimate();

            current_x = currentPose.getX();
            current_y = currentPose.getY();
            
            current_angle = 0 - currentPose.getHeading() - zero_angle;
        } //current_angle: same angle system as left/right stick angle

        double distance_factor;
        double offset;

        if (left_stick_magnitude != 0) {
            distance_factor = left_stick_magnitude;

            if (locked_motion) {
                offset = modifiedAngle(left_stick_angle - current_angle);
            } else {
                offset = left_stick_angle;
            }

            if (useRoadRunner) {
                target_x = current_x;
                target_y = current_y;
            }

        } else {

            distance_factor = Math.sqrt((current_x - target_x) * (current_x - target_x) + (current_y - target_y) * (current_y - target_y)) * distance_weight_two;
                //zero by default if not using RoadRunner :)

            double line_angle = (target_x > current_x) ? Math.atan(((float) target_y - (float) current_y)/((float) target_x - (float) current_x)) :
            (target_x < current_x) ? Math.PI + Math.atan(((float) target_y - (float) current_y)/((float) target_x - (float) current_x)) :
            (target_y > current_y) ? Math.PI / 2.0 : -Math.PI / 2.0;
            
            line_angle += Math.PI / 2.0;
            offset = modifiedAngle(line_angle - current_angle);
        }

        double turning_factor = 0;

        if (right_stick_magnitude != 0) {
            if (locked_rotation) {
                target_angle = right_stick_angle;
            } else { //if we're driving normally
                target_angle = current_angle;
                turning_factor = gamepad.right_stick_x;
            }
        } //target angle remains constant if we aren't turning manually

        drive(turning_factor, distance_factor, offset, speedFactor);
    }

    public void drive(double turning_factor, double distance_factor, double offset, double speedFactor) {
        double correction = getCorrection();
        turning_factor *= turning_weight;
        turning_factor += correction;
        distance_factor *= distance_weight;
        double[] power = new double[4];
        for (int i = 0; i < 4; i++) {
            power[i] = turning_factor * ((i > 1) ? -1 : 1) - distance_factor * (Math.cos(offset) + Math.sin(offset) * (i % 2 == 1 ? 1 : -1) / strafe);
        }
        double maximum = Math.max(1, Math.max(Math.max(Math.abs(power[0]), Math.abs(power[1])), Math.max(Math.abs(power[2]), Math.abs(power[3]))));
        for (int i = 0; i < 4; i++) {
            robot.wheel_list[i].setPower(power[i] / maximum / speedFactor);
        }
    }
    
    public double getCorrection() {
        current_time = System.currentTimeMillis();
        current_error = modifiedAngle(target_angle - current_angle);

        double p = current_error;
        double d = (current_error - previous_error) / (current_time - previous_time);

        previous_error = current_error;
        previous_time = current_time;

        return p_weight * p + d_weight * d;
    }

    public double modifiedAngle(double radians) {
        while (radians > Math.PI) {
            radians -= 2 * Math.PI;
        }
        while (radians < 0 - Math.PI) {
            radians += 2 * Math.PI;
        }
        return radians;
    }

    //Initialization

    public void new_keybind(String motor, String button, Object modifier1, Object modifier2, Object modifier3) {
        Object temp2;
        if (!keybinds.containsKey(motor)) throw new IllegalArgumentException("You misspelled " + motor + " - make sure its exactly as it's spelled in dc motor list or servo list, or it's \"goto\". Idiot");
        if (!(keys.contains(button))) throw new IllegalArgumentException("You misspelled " + button + "  - make sure its exactly as it's spelled in keys. ");
        if (keybinds.get(motor).contains((Object) button)) throw new IllegalArgumentException("You can't have \"" + button + "\" have 2 different functions for the same motor and button combination. The motor is " +  motor + ". ");

        if (dc_motor_names.contains(motor) || cr_servo_names.contains(motor) || servo_names.contains(motor)) {
            if (button_types[keys.indexOf(button)] == null) {
                button_types[keys.indexOf(button)] = (String) modifier1;
                if (modifier1.equals("cycle")) {
                    button_types[keys.indexOf(button)] = "button";
                }
            } else if (!button_types[keys.indexOf(button)].equals((String) modifier1)) {
                //if already set to something different
                if (!(modifier1.equals("cycle") && button_types[keys.indexOf(button)].equals("button"))) {
                    throw new IllegalArgumentException("A button cannot have 2 types; however, you are setting \"" + button +
                            "\" to be both a " + button_types[keys.indexOf(button)] + " and a button. (\"goto\" is, by default, a button) ");
                }
            }

            if (((String) modifier1).equals("button") || ((String) modifier1).equals("cycle")) {
                if (cr_servo_names.contains(motor)) throw new IllegalArgumentException("You can't set positions to Continuous Rotation Servos");
                try {
                    temp2 = (int) modifier2;
                } catch(ClassCastException e) {
                    throw new IllegalArgumentException("Increments have to be by an integer amount. Error was on key " + button + ". ");
                }
                try {
                    temp2 = (double[]) modifier3;
                } catch(ClassCastException e) {
                    throw new IllegalArgumentException("The positions list has to be one of doubles, even if it's a dc motor positions list. Error was on key " + button + ". ");
                }
            } else if (((String) modifier1).equals("toggle") || ((String) modifier1).equals("default")) {
                if (((String) modifier1).equals("default") && (keys.indexOf((String) button) > 19)) {
                    try {
                        temp2 = (double) modifier2;
                        temp2 = (double) modifier3;
                    } catch(ClassCastException e) {
                        throw new IllegalArgumentException("Power has to be a double. Error was on key " + button + ". ");
                    }
                    if ((Math.max(Math.abs((double) modifier2), Math.abs((double) modifier3)) > 1) && dc_motor_names.contains(motor)) {
                        throw new IllegalArgumentException("DC Motor Power has to be between -1.0 and 1.0. Error was on key " + button + ". ");
                    }
                } else if (servo_names.contains(motor)) { //servo, default or toggle
                    try {
                        temp2 = (double) modifier2;
                    } catch(ClassCastException e) {
                        throw new IllegalArgumentException("Power has to be a double. Error was on key " + button + ". ");
                    }
                } else {
                    try {
                        temp2 = (String) modifier2;
                        if (!((String) modifier2).equals("normal") && !((String) modifier2).equals("gradient")) {
                            throw new ClassCastException();
                        }
                    } catch(ClassCastException e) {
                        throw new IllegalArgumentException("Button type has to be \"normal\" or \"gradient\". Error was on key " + button + ". ");
                    }
                    try {
                        temp2 = (double) modifier3;
                        if (Math.abs((double) modifier3) > 1) {
                            throw new ClassCastException();
                        }
                    } catch(ClassCastException e) {
                        throw new IllegalArgumentException("Power has to be a double between -1.0 and 1.0. Error was on key " + button + ". ");
                    }
                }
            } else {
                throw new IllegalArgumentException("You misspelled " + modifier1 + " in key " + button + " - make sure its \"default\", \"button\", \"cycle\" or \"toggle\".");
            }
        } else { //goto
            if (button_types[keys.indexOf((String) button)] == null) {
                button_types[keys.indexOf((String) button)] = "button";
            } else if (!button_types[keys.indexOf((String) button)].equals("button")) {
                throw new IllegalArgumentException("A button cannot have 2 types; however, you are setting \"" + button +
                        "\" to be both a " + button_types[keys.indexOf((String) button)] + " and a button. (\"goto\" is, by default, a button) ");
            }
            try {
                temp2 = (double) modifier1;
                temp2 = (double) modifier2;
            } catch(ClassCastException e) {
                throw new IllegalArgumentException("Target x/y must be doubles (integers are fine as well)");
            }
            try {
                temp2 = (double) modifier3;
            } catch(ClassCastException e) {
                if (!temp2.equals("none")) {
                    throw new IllegalArgumentException("target angle must be a double, or be labeled as \"none\"");
                }
            }
        }
        keybinds.get(motor).add((Object) button);
        keybinds.get(motor).add(modifier1);
        keybinds.get(motor).add(modifier2);
        keybinds.get(motor).add(modifier3);
    }

    public void set_button_types() {
        for (int i = 0; i < 27; i++) {
            if (button_types[i] == null) {
                button_types[i] = "default";
            }
        }
    }

    public void resetZeroAngle() {
        if (useRoadRunner) {
            zero_angle = 0 - position_tracker.getPoseEstimate().getHeading();
        } else if (usePID) {
            zero_angle = 0 - robot.getAngle();
        }
    }

    public void setZeroAngle(double angle) {
        zero_angle = 0 - Math.toRadians(angle);
    }

    public void initializeRoadRunner(double x, double y, double angle, StandardTrackingWheelLocalizer localizer) {
        position_tracker = localizer;
        position_tracker.setPoseEstimate(new Pose2d(x, y, Math.toRadians(angle)));
        current_x = x;
        current_y = y;
        current_angle = Math.toRadians(angle);
        target_x = x;
        target_y = y;
        target_angle = Math.toRadians(angle);
        zero_angle -= Math.toRadians(angle);
    }

    //RoadRunner

    public double angle() { return modifiedAngle(0 - zero_angle - current_angle); }

    public double[][] robot_hitbox() {
        return new double[][] {
                {current_x + robot_length * Math.cos(angle()) - robot_width * Math.sin(angle()), current_y + robot_length * Math.sin(angle()) + robot_width * Math.cos(angle())},
                {current_x + robot_length * Math.cos(angle()) + robot_width * Math.sin(angle()), current_y + robot_length * Math.sin(angle()) - robot_width * Math.cos(angle())},
                {current_x - robot_length * Math.cos(angle()) + robot_width * Math.sin(angle()), current_y - robot_length * Math.sin(angle()) - robot_width * Math.cos(angle())},
                {current_x - robot_length * Math.cos(angle()) - robot_width * Math.sin(angle()), current_y - robot_length * Math.sin(angle()) + robot_width * Math.cos(angle())}
        };
    }

    public double distance_from(double[] point) {
        return Math.sqrt((point[0] - current_x) * (point[0] - current_x) + (point[1] - current_y) * (point[1] - current_y));
    }

    public boolean point_above_line(double[] point, double[][] line) { //definition of "above": y above line, or if vertical, then x value greater
        //point format:{x, y}
        //line format: {{x1, y1}, {x2, y2}}
        if (line[0][0] == line[1][0]) {
            return point[0] > line[0][0];
        } else if (point[0] == line[0][0]) {
            return point[1] > line[0][1];
        } else if (point[0] > line[0][0]) {
            return (point[1] - line[0][1]) / (point[0] - line[0][0]) > (line[1][1] - line[0][1]) / (line[1][0] - line[0][0]);
        } else {
            return (point[1] - line[0][1]) / (point[0] - line[0][0]) < (line[1][1] - line[0][1]) / (line[1][0] - line[0][0]);
        }
    }

    public boolean point_on_line(double[] point, double[][] line) {
        if (Math.abs(line[1][0] - line[0][0]) == 0) {
            return (Math.abs(line[0][0] - point[0]) < 0.01);
        } else if (Math.abs(point[0] - line[0][0]) == 0) {
            return false;
        } else {
            return (point[1] - line[0][1]) / (point[0] - line[0][0]) == (line[1][1] - line[0][1]) / (line[1][0] - line[0][0]);
        }
    }

    public boolean intersect(double[][] line1, double[][] line2) {
        double[][] line_1 = line1;
        double[][] line_2 = line2;

        if ((Math.max(line_1[0][0], line_1[1][0]) < Math.min(line_2[0][0], line_2[1][0])) || (Math.min(line_1[0][0], line_1[1][0]) > Math.max(line_2[0][0], line_2[1][0])) ||
                (Math.max(line_1[0][1], line_1[1][1]) < Math.min(line_2[0][1], line_2[1][1])) || (Math.min(line_1[0][1], line_1[1][1]) > Math.max(line_2[0][1], line_2[1][1]))) {
            return false;
        }

        if (line_1[0][0] == line_1[1][0]) {
            line_1[1][0] += 0.01;
        }
        if (line_2[0][0] == line_2[1][0]) {
            line_2[1][0] += 0.01;
        }

        if (point_on_line(line_1[0], line_2) || point_on_line (line_1[1], line_2) || point_on_line(line_2[0], line_1) || point_on_line (line_2[1], line_1)) {
            return false;
        } else {
            return (((point_above_line(line_1[0], line_2) != point_above_line(line_1[1], line_2))) && ((point_above_line(line_2[0], line_1) != point_above_line(line_2[1], line_1))));
        }
    }

    public boolean point_inside_polygon(double[] point, double[][] polygon, int accuracy) { //we want the inside-ness to be the same for every line
        double min_x = 10000000;
        double min_y = 10000000;
        double max_x = -10000000;
        double max_y = -10000000;
        int intersections;
        double[][] radial_line = new double[2][2];
        double[][] next_line = new double[2][2];

        for (double[] i : polygon) {
            min_x = Math.min(min_x, i[0]);
            max_x = Math.max(max_x, i[0]);
            min_y = Math.min(min_y, i[1]);
            max_y = Math.max(max_y, i[1]);
        }
        double length = Math.sqrt((max_x - min_x) * (max_x - min_x) + (max_y - min_y) * (max_y - min_y));

        for (int i = 0; i < accuracy; i++) {
            intersections = 0;
            radial_line[0][0] = point[0];
            radial_line[0][1] = point[1];
            radial_line[1][0] = point[0] + length * Math.cos(2.0 * Math.PI / (double) accuracy * (double) i);
            radial_line[1][1] = point[1] + length * Math.sin(2.0 * Math.PI / (double) accuracy * (double) i);

            for (int j = 0; j < polygon.length; j++) { //add 1 if intersects line, 0.5 for each endpoint it touches
                next_line[0] = polygon[j];
                next_line[1] = polygon[(j + 1) % polygon.length];
                if (point_on_line(polygon[j], radial_line)) {
                    intersections += 1;
                }
                if (point_on_line(polygon[(j + 1) % polygon.length], radial_line)) {
                    intersections += 1;
                }
                if (intersect(radial_line, next_line)) {
                    intersections += 2;
                }
            }
            if (intersections % 4 != 2) {
                return false;
            }
        }
        return true;
    }

    public boolean completely_inside(double[][] polygon1, double[][] polygon2, int accuracy) {
        for (double[] point : polygon1) {
            if (!point_inside_polygon (point, polygon2, accuracy))
                return false;
        }
        return true;
    }

    public boolean shells_intersect(double[][] polygon1, double[][] polygon2) { //only seeing if the outer shells interesect each other
        for (int i = 0; i < polygon1.length; i++) {
            for (int j = 0; j < polygon2.length; j++) {
                if (intersect(new double[][] {polygon1[i], polygon1[(i + 1) % polygon1.length]}, new double[][] {polygon2[j], polygon2[(j + 1) % polygon2.length]}))
                    return true;
            }
        }
        return false;
    }

    public boolean polygons_intersect(double[][] polygon1, double[][] polygon2, int accuracy) {
        return (shells_intersect(polygon1, polygon2) || (completely_inside(polygon1, polygon2, accuracy) || completely_inside(polygon2, polygon1, accuracy)));
    }

    public boolean inside_polygon(double[][] polygon) {
        return completely_inside(robot_hitbox(), polygon, 6);
    }

    public boolean robot_on_point(double[] point) {
        return point_inside_polygon(point, robot_hitbox(), 6);
    }

    public boolean facing_polygon(double[][] polygon, double max_length, double offset) {
        double[][] radial_line = {{current_x, current_y}, {current_x + max_length * Math.cos(angle() + Math.toRadians(offset)), current_y + max_length * Math.sin(angle() + Math.toRadians(offset))}};
        double[][] next_line = new double[2][2];

        for (int j = 0; j < polygon.length; j++) { //add 1 if intersects line, 0.5 for each endpoint it touches
            next_line[0] = polygon[j];
            next_line[1] = polygon[(j + 1) % polygon.length];
            if (intersect(radial_line, next_line)) {
                return true;
            }
        }
        return false;
    }
}