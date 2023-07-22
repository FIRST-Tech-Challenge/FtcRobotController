package org.firstinspires.ftc.teamcode.Components.RFModules.Devices;

import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.logger;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.op;

import java.util.ArrayList;
import java.util.HashMap;

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
     * @param value
     * @param name
     * @param action
     */
    public void readGamepad(float value, String name, String action) {
        if (value != floatMap.get(name)) {
            logger.log("/RobotLogs/GeneralRobot", "Gamepad," + name + "," + action + ": " + value, true);
            floatMap.replace(name, value);
        }
    }

    /**
     * readgamepad for floats, logs & changes corresponding element in hashmap
     * @param value
     * @param name
     * @param action
     * @return
     */
    public boolean readGamepad(boolean value, String name, String action) {
        if (value != booleanMap.get(name)) {
            logger.log("/RobotLogs/GeneralRobot", "Gamepad," + name + "," + action + ": " + value, true);
            booleanMap.replace(name, value);
            return value;
        }
        return false;
    }

    /**
     * a lot of if statements to check if buttons have been let go
     * @return
     */
    public boolean updateSequence() {
        boolean updated = false;
        if (op.gamepad1.dpad_down) {
            if (!booleanMap.get("gamepad1_dpad_down")) {
                seq.add(3);
                booleanMap.replace("gamepad1_dpad_down", true);
                updated = true;
            }
        }
        else if (!op.gamepad1.dpad_down) {
            booleanMap.replace("gamepad1_dpad_down", false);
        }
        if (op.gamepad1.dpad_right) {
            if (!booleanMap.get("gamepad1_dpad_right")) {
                seq.add(2);
                booleanMap.replace("gamepad1_dpad_right", true);
                updated = true;
            }
        }
        else if (!op.gamepad1.dpad_right) {
            booleanMap.replace("gamepad1_dpad_right", false);
        }
        if (op.gamepad1.dpad_up) {
            if (!booleanMap.get("gamepad1_dpad_up")) {
                seq.add(1);
                booleanMap.replace("gamepad1_dpad_up", true);
                updated = true;
            }
        }
        else if (!op.gamepad1.dpad_up) {
            booleanMap.replace("gamepad1_dpad_up", false);
        }
        if (op.gamepad1.dpad_left) {
            if (!booleanMap.get("gamepad1_dpad_left")) {
                seq.add(4);
                booleanMap.replace("gamepad1_dpad_left", true);
                updated = true;
            }
        }
        else if (!op.gamepad1.dpad_left) {
            booleanMap.put("gamepad1_dpad_left", false);
        }
        if (op.gamepad1.x) {
            if (!booleanMap.get("gamepad1_x")) {
                seq.add(5);
                booleanMap.replace("gamepad1_x", true);
                updated = true;
            }
        }
        else if (!op.gamepad1.x) {
            booleanMap.replace("gamepad1_x", false);
        }
        if (op.gamepad1.b) {
            if (!booleanMap.get("gamepad1_b")) {
                seq.add(6);
                booleanMap.replace("gamepad1_b", true);
                updated = true;
            }
        }
        else if (!op.gamepad1.b) {
            booleanMap.replace("gamepad1_b", false);
        }
        return updated;
    }

    /**
     * returns sequence of key presses
     * @return
     */
    public ArrayList<Integer> getSequence() {
        return seq;
    }

    /**
     * get rid of first element in arraylist
     */
    public void removeSequenceElement() {
        if(!seq.isEmpty()) {
            seq.remove(0);
        }
    }

    /**
     * clear arraylist
     */
    public void clearSequence(){
        seq.clear();
    }
}
