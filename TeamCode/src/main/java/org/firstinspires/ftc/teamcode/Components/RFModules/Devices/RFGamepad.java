package org.firstinspires.ftc.teamcode.Components.RFModules.Devices;

import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.logger;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.op;

import java.util.ArrayList;
import java.util.HashMap;

public class RFGamepad {
    HashMap<String, Boolean> booleanMap = new HashMap<>();
    HashMap<String, Float> floatMap = new HashMap<>();
    ArrayList<Integer> seq = new ArrayList<>();

    public RFGamepad() {
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

    public void readGamepad(float value, String name, String action) {
        if (value != floatMap.get(name)) {
            logger.log("/RobotLogs/GeneralRobot", "Gamepad," + name + "," + action + ": " + value, true);
            floatMap.replace(name, value);
        }
    }

    public boolean readGamepad(boolean value, String name, String action) {
        if (value != booleanMap.get(name)) {
            logger.log("/RobotLogs/GeneralRobot", "Gamepad," + name + "," + action + ": " + value, true);
            booleanMap.replace(name, value);
            return value;
        }
        return false;
    }

    //    public boolean isReleased(String name){
//        boolean val = false;
//        if(booleanMap.get(name) == false){
//            val = true;
//        }
//        else if(booleanMap.get(name) == true){
//            val = false;
//        }
//        return val;
//    }
    //1 = down, 2 = right, 3 = up, 4 = left

    //    ArrayList<Integer> old = new ArrayList<>();
//    int newVal;
//    double timeOfPress = 0;
    // Boolean map of status of all buttons
    // Take in a button, update that corresponding status with the status of button
    //    -
    //
    //
    public boolean updateSequence() {
        boolean updated = false;
        if (op.gamepad1.dpad_down) {
            if (!booleanMap.get("gamepad1_dpad_down")) {
                seq.add(3);
                booleanMap.replace("gamepad1_dpad_down", true);
                updated = true;
            }
            //timeOfPress = op.getRuntime();
        } else if (!op.gamepad1.dpad_down) {
            booleanMap.replace("gamepad1_dpad_down", false);
        }
        if (op.gamepad1.dpad_right) {
            if (!booleanMap.get("gamepad1_dpad_right")) {
                seq.add(2);
                booleanMap.replace("gamepad1_dpad_right", true);
                updated = true;
            }
            //timeOfPress = op.getRuntime();
        } else if (!op.gamepad1.dpad_right) {
            booleanMap.replace("gamepad1_dpad_right", false);
        }
        if (op.gamepad1.dpad_up) {
            if (!booleanMap.get("gamepad1_dpad_up")) {
                seq.add(1);
                booleanMap.replace("gamepad1_dpad_up", true);
                updated = true;
            }
            //timeOfPress = op.getRuntime();
        } else if (!op.gamepad1.dpad_up) {
            booleanMap.replace("gamepad1_dpad_up", false);
        }
        if (op.gamepad1.dpad_left) {
            if (!booleanMap.get("gamepad1_dpad_left")) {
                seq.add(4);
                booleanMap.replace("gamepad1_dpad_left", true);
                updated = true;
            }
            //timeOfPress = op.getRuntime();
        } else if (!op.gamepad1.dpad_left) {
            booleanMap.put("gamepad1_dpad_left", false);
        }
        if (op.gamepad1.x) {
            if (!booleanMap.get("gamepad1_x")) {
                seq.add(5);
                booleanMap.replace("gamepad1_x", true);
                updated = true;
            }
            //timeOfPress = op.getRuntime();
        } else if (!op.gamepad1.x) {
            booleanMap.replace("gamepad1_x", false);
        }
        if (op.gamepad1.b) {
            if (!booleanMap.get("gamepad1_b")) {
                seq.add(6);
                booleanMap.replace("gamepad1_b", true);
                updated = true;
            }
            //timeOfPress = op.getRuntime();
        } else if (!op.gamepad1.b) {
            booleanMap.replace("gamepad1_b", false);
        }
        return updated;
    }

    public ArrayList<Integer> getSequence() {
        return seq;
    }

    public void removeSequenceElement() {
        if(!seq.isEmpty()) {
            seq.remove(0);
        }
    }
    public void clearSequence(){
        seq.clear();
    }
}
