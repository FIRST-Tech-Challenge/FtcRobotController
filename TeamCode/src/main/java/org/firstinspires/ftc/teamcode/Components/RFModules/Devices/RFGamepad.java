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
    public RFGamepad(LinearOpMode op){
        logger.createFile("gamepad", "Time Name Value");
        booleanMap.put("gamepad1_y", false);
        booleanMap.put("gamepad1_x", false);
        booleanMap.put("gamepad1_a", false);
        booleanMap.put("gamepad1_b", false);
        booleanMap.put("gamepad1_left_bumper", false);
        booleanMap.put("gamepad1_right_bumper", false);
        booleanMap.put("gamepad1_dpad_left", false);
        booleanMap.put("gamepad1_dpad_right", false);
        floatMap.put("gamepad1_left_stick_y", 0f);
        floatMap.put("gamepad1_right_stick_x", 0f);
        floatMap.put("gamepad1_right_stick_y", 0f);
    }
    public void readGamepad(float value, String name, String action){
        if(value != floatMap.get(name)){
            logger.log("gamepad", " " + name + " | " + action + ": " + value);
            floatMap.put(name, value);
        }
    }
    public void readGamepad(boolean value, String name, String action){
        if(value != booleanMap.get(name)){
            logger.log("gamepad", " " + name + " | " + action + ": " + value);
            booleanMap.put(name, value);
        }
    }
}
