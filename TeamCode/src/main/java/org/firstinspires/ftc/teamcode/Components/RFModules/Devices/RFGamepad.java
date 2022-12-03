package org.firstinspires.ftc.teamcode.Components.RFModules.Devices;

import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.logger;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.op;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;

import org.firstinspires.ftc.teamcode.Robots.BasicRobot;
import org.firstinspires.ftc.teamcode.Components.Logger;

import java.util.ArrayList;
import java.util.HashMap;

public class RFGamepad{
    HashMap<String, Boolean> booleanMap=  new HashMap<>();
    HashMap<String, Float> floatMap= new HashMap<>();
    public RFGamepad(){
        booleanMap.put("gamepad1_y", false);
        booleanMap.put("gamepad1_x", false);
        booleanMap.put("gamepad1_a", false);
        booleanMap.put("gamepad1_b", false);
        booleanMap.put("gamepad1_left_bumper", false);
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
    public void readGamepad(float value, String name, String action){
        if(value != floatMap.get(name)){
            logger.log("/RobotLogs/GeneralRobot", "Gamepad," + name + "," + action + ": " + value, true);
            floatMap.put(name, value);
        }
    }
    public void readGamepad(boolean value, String name, String action){
        if(value != booleanMap.get(name)){
            logger.log("/RobotLogs/GeneralRobot", "Gamepad," + name + "," + action + ": " + value, true);
            booleanMap.put(name, value);
        }
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
    ArrayList<Integer> seq = new ArrayList<>();
//    ArrayList<Integer> old = new ArrayList<>();
//    int newVal;
//    double timeOfPress = 0;
    // Boolean map of status of all buttons
    // Take in a button, update that corresponding status with the status of button
    //    -
    //
    //
    public ArrayList<Integer> getSequence() {
//        for(int i=0; i<seq.size(); i++){
//            old.set(0, seq.get(i));
//        }
        if (op.gamepad1.dpad_down){
            if(!booleanMap.get("gamepad1_dpad_down")){
                seq.add(3);
                booleanMap.put("gamepad1_dpad_down", true);
                //TODO: pretty sure if u keep putting more things in the booleanMap ull build up duplicates, looks like
                // there is a replace function
            }
            //timeOfPress = op.getRuntime();
        }
        else if(!op.gamepad1.dpad_down){
            booleanMap.put("gamepad1_dpad_down", false);
        }
        if (op.gamepad1.dpad_right){
            if(!booleanMap.get("gamepad1_dpad_right")){
                seq.add(2);
                booleanMap.put("gamepad1_dpad_right", true);
            }
            //timeOfPress = op.getRuntime();
        }
        else if(!op.gamepad1.dpad_right){
            booleanMap.put("gamepad1_dpad_right", false);
        }
        if (op.gamepad1.dpad_up){
            if(!booleanMap.get("gamepad1_dpad_up")){
                seq.add(1);
                booleanMap.put("gamepad1_dpad_up", true);
            }
            //timeOfPress = op.getRuntime();
        }
        else if(!op.gamepad1.dpad_up){
            booleanMap.put("gamepad1_dpad_up", false);
        }
        if (op.gamepad1.dpad_left){
            if(!booleanMap.get("gamepad1_dpad_left")){
                seq.add(4);
                booleanMap.put("gamepad1_dpad_left", true);
            }
            //timeOfPress = op.getRuntime();
        }
        else if(!op.gamepad1.dpad_left){
            booleanMap.put("gamepad1_dpad_left", false);
        }
        if (op.gamepad1.x){
            if(!booleanMap.get("gamepad1_x")){
                seq.add(5);
                booleanMap.put("gamepad1_x", true);
            }
            //timeOfPress = op.getRuntime();
        }
        else if(!op.gamepad1.x){
            booleanMap.put("gamepad1_x", false);
        }
        if (op.gamepad1.b){
            if(!booleanMap.get("gamepad1_b")){
                seq.add(6);
                booleanMap.put("gamepad1_b", true);
            }
            //timeOfPress = op.getRuntime();
        }
        else if(!op.gamepad1.b){
            booleanMap.put("gamepad1_b", false);
        }




//            if (op.gamepad1.dpad_right) {
//                seq.add(2);
//                //timeOfPress = op.getRuntime();
//            }
//            if (op.gamepad1.dpad_up) {
//                seq.add(3);
//                //timeOfPress = op.getRuntime();
//            }
//            if (op.gamepad1.dpad_left) {
//                seq.add(4);
//                //timeOfPress = op.getRuntime();
//            }
//            if (op.gamepad1.x) {
//                seq.add(5);
//                //timeOfPress = op.getRuntime();
//            }
//            if (op.gamepad1.b) {
//                seq.add(6);
//                //timeOfPress = op.getRuntime();
//            }
//        }
//        if(seq.size() == old.size()){
//            newVal = 0;
//        }
//        else if(seq.size() > old.size()){
//            newVal = seq.get(seq.size()-1);
//        }
//        return newVal;
        return seq;
    }

    public void removeSequenceElement () {
        seq.remove(0);
    }
}
