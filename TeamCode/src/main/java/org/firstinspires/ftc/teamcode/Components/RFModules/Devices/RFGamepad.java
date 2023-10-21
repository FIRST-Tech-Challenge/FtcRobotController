package org.firstinspires.ftc.teamcode.Components.RFModules.Devices;

import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.LOGGER;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.logger;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.op;

import org.firstinspires.ftc.teamcode.Components.RFModules.System.RFLogger;

import java.util.ArrayList;
import java.util.HashMap;

/**
 * Harry
 */
public class RFGamepad {
    //hashmaps for storing states and arraylist for output
    private HashMap<String, Boolean> booleanMap = new HashMap<>();
    private HashMap<String, Float> floatMap = new HashMap<>();
    private ArrayList<Integer> seq = new ArrayList<>();

    public RFGamepad() {
        //all the gamepad buttons are created and set to their default states, false & 0
        booleanMap.put("gamepad1_y", false);
        booleanMap.put("gamepad1_x", false);
        booleanMap.put("gamepad1_a", false);
        booleanMap.put("gamepad1_b", false);
        booleanMap.put("gamepad1_left_bumper", false);
        booleanMap.put("gamepad1_left_trigger", false);
        booleanMap.put("gamepad1_right_bumper", false);
        booleanMap.put("gamepad1_dpad_left", false);
        booleanMap.put("gamepad1_dpad_right", false);
        booleanMap.put("gamepad1_dpad_up", false);
        booleanMap.put("gamepad1_dpad_down", false);
        floatMap.put("gamepad1_left_stick_x", 0f);
        floatMap.put("gamepad1_left_stick_y", 0f);
        floatMap.put("gamepad1_right_stick_x", 0f);
        floatMap.put("gamepad1_right_stick_y", 0f);
        floatMap.put("gamepad1_right_trigger", 0f);
        floatMap.put("gamepad1_left_trigger", 0f);
        booleanMap.put("gamepad2_y", false);
        booleanMap.put("gamepad2_x", false);
        booleanMap.put("gamepad2_a", false);
        booleanMap.put("gamepad2_b", false);
        booleanMap.put("gamepad2_left_bumper", false);
        booleanMap.put("gamepad2_right_bumper", false);
        booleanMap.put("gamepad2_dpad_left", false);
        booleanMap.put("gamepad2_dpad_right", false);
        booleanMap.put("gamepad2_dpad_up", false);
        booleanMap.put("gamepad2_dpad_down", false);
        floatMap.put("gamepad2_left_stick_x", 0f);
        floatMap.put("gamepad2_left_stick_y", 0f);
        floatMap.put("gamepad2_right_stick_x", 0f);
        floatMap.put("gamepad2_right_stick_y", 0f);
        floatMap.put("gamepad2_right_trigger", 0f);
        floatMap.put("gamepad2_left_trigger", 0f);
    }

    /**
     * readgamepad for floats, logs & changes corresponding element in hashmap
     *
     * @param value
     * @param name
     * @param action
     */
    public void readGamepad(float value, String name, String action) {
        if (value != floatMap.get(name)) {
            LOGGER.log(name + " pressed, " + action);
            floatMap.replace(name, value);
        }
    }

    /**
     * readgamepad for floats, logs & changes corresponding element in hashmap
     *
     * @param value
     * @param name
     * @param action
     * @return
     */
    public boolean readGamepad(boolean value, String name, String action) {
        if (value != booleanMap.get(name)) {
            if (value)
                LOGGER.log(name + " pressed, " + action);
            else
                LOGGER.log(name + " released, " + action);
            booleanMap.replace(name, value);
            return value;
        }
        return false;
    }


    /**
     * returns sequence of key presses
     *
     * @return
     */
    public ArrayList<Integer> getSequence() {
        return seq;
    }

    /**
     * get rid of first element in arraylist
     */
    public void removeSequenceElement() {
        if (!seq.isEmpty()) {
            seq.remove(0);
        }
    }

    /**
     * clear arraylist
     */
    public void clearSequence() {
        seq.clear();
    }
}
