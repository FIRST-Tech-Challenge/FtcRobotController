package org.firstinspires.ftc.team6220_PowerPlay;

import java.lang.reflect.Array;
import java.util.ArrayList;
import java.util.Stack;
import java.util.logging.Filter;

abstract public class BaseTeleOp extends BaseOpMode {
    public Float[] GamePadInputAVG() {
        //Initialization for all variables
        Stack<Float> XAVGPos_LS = new Stack<Float>();
        Stack<Float> YAVGPos_LS = new Stack<Float>();
        Stack<Float> XAVGPos_RS = new Stack<Float>();
        Stack<Float> YAVGPos_RS = new Stack<Float>();
        Float[] AllStickAVGS = {null, null, null, null};

        //Scuffed af
        if (XAVGPos_LS.size() < 5) {
            XAVGPos_LS.push(gamepad1.left_stick_x);
            YAVGPos_LS.push(gamepad1.left_stick_y);
            XAVGPos_RS.push(gamepad1.right_stick_x);
            YAVGPos_RS.push(gamepad1.right_stick_y);
        } else {
            XAVGPos_LS.pop();
            YAVGPos_LS.pop();
            XAVGPos_RS.pop();
            YAVGPos_RS.pop();
        }
        //Actual averaging, still kinda sus
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

        //Stick deadzones math:
        //FilteredInput[0] = Left stick X pos
        //FilteredInput[1] = Left stick Y pos
        //FilteredInput[2] = Right stick X pos
        //FilteredInput[3] = Right stick Y pos

        //Driving / pivoting, not sure how we want it to be structured so this can be fixed later
        if(Math.atan2(FilteredInput[0], FilteredInput[1]) < 20){
            DriveRobot(FilteredInput[0], 0, FilteredInput[2]);
        }else if(Math.atan2(FilteredInput[1], FilteredInput[0]) < 20){
            DriveRobot(0, FilteredInput[1], FilteredInput[2]);
        }else{
            DriveRobot(FilteredInput[0], FilteredInput[1], FilteredInput[2]);
        }

    }

}
