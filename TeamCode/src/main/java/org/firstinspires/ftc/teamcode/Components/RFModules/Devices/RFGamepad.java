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
}
