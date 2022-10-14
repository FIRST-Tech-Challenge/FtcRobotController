package org.firstinspires.ftc.team6220_PowerPlay;

import org.firstinspires.ftc.team6220_PowerPlay.ResourceClasses.Constants;



import java.lang.reflect.Array;
import java.util.ArrayList;
import java.util.LinkedList;
import java.util.Queue;
import java.util.Stack;
import java.util.logging.Filter;

abstract public class BaseTeleOp extends BaseOpMode {
    public Float[] GamePadInputAVG() {

        Queue<String> e = new LinkedList<String>();
        
        //Initialization for all variables, the stack type is used here for easy adding and removing from the front and back of the list.
        Queue<Float> XAVGPos_LS = new LinkedList<>();
        Queue<Float> YAVGPos_LS = new LinkedList<>();
        Queue<Float> XAVGPos_RS = new LinkedList<>();
        Queue<Float> YAVGPos_RS = new LinkedList<>();
        Float[] AllStickAVGS = {0f, 0f, 0f, 0f};

        //This pushes the last recorded stick position if the stack size is below 5.
        //It removes the last element and pushes the last recorded stick position is the stack size is above 5
        if (XAVGPos_LS.size() < 5) {
            XAVGPos_LS.add(gamepad1.left_stick_x);
            YAVGPos_LS.add(gamepad1.left_stick_y);
            XAVGPos_RS.add(gamepad1.right_stick_x);
            YAVGPos_RS.add(gamepad1.right_stick_y);
        } else {
            XAVGPos_LS.remove();
            YAVGPos_LS.remove();
            XAVGPos_RS.remove();
            YAVGPos_RS.remove();
            XAVGPos_LS.add(gamepad1.left_stick_x);
            YAVGPos_LS.add(gamepad1.left_stick_y);
            XAVGPos_RS.add(gamepad1.right_stick_x);
            YAVGPos_RS.add(gamepad1.right_stick_y);
        }
        //Loops through the stack and outputs a list of averaged x and y stick positions
        for(int iterator = 0; iterator < XAVGPos_LS.size(); iterator++){
            AllStickAVGS[0] = (XAVGPos_LS.get(iterator) + AllStickAVGS[0])/iterator;
            AllStickAVGS[1] = (YAVGPos_LS.get(iterator) + AllStickAVGS[1])/iterator;
            AllStickAVGS[2] = (XAVGPos_RS.get(iterator) + AllStickAVGS[2])/iterator;
            AllStickAVGS[3] = (YAVGPos_RS.get(iterator) + AllStickAVGS[3])/iterator;
        }
        return (AllStickAVGS);
    }

    public void TeleOpDrive () {
        Float[] FilteredInput = GamePadInputAVG();

        //Stick deadzones:
        //FilteredInput[0] = Left stick X pos
        //FilteredInput[1] = Left stick Y pos
        //FilteredInput[2] = Right stick X pos
        //FilteredInput[3] = Right stick Y pos

        //Driving / pivoting, not sure how we want it to be structured so this can be fixed later
        if(Math.atan2(FilteredInput[0], FilteredInput[1]) < Constants.deadzoneRange){
            driveRobot(FilteredInput[0], 0, FilteredInput[2]);
        }else if(Math.atan2(FilteredInput[1], FilteredInput[0]) < Constants.deadzoneRange){

            driveRobot(0, FilteredInput[1], FilteredInput[2]);
        } else {
            driveRobot(FilteredInput[0], FilteredInput[1], FilteredInput[2]);
        }

    }

}

