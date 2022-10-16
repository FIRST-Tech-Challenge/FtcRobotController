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
        ArrayList<Float> XAVGPos_LS = new ArrayList<>();
        ArrayList<Float> YAVGPos_LS = new ArrayList<>();
        ArrayList<Float> XAVGPos_RS = new ArrayList<>();
        ArrayList<Float> YAVGPos_RS = new ArrayList<>();
        Float[] AllStickAVGS = {0f, 0f, 0f, 0f};

        //This pushes the last recorded stick position if the stack size is below 5.
        //It removes the last element and pushes the last recorded stick position is the stack size is above 5
        if (XAVGPos_LS.size() < 5) {
            XAVGPos_LS.add(gamepad1.left_stick_x);
            YAVGPos_LS.add(gamepad1.left_stick_y);
            XAVGPos_RS.add(gamepad1.right_stick_x);
            YAVGPos_RS.add(gamepad1.right_stick_y);
        } else {
            XAVGPos_LS.remove(0);
            YAVGPos_LS.remove(0);
            XAVGPos_RS.remove(0);
            YAVGPos_RS.remove(0);
            XAVGPos_LS.add(gamepad1.left_stick_x);
            YAVGPos_LS.add(gamepad1.left_stick_y);
            XAVGPos_RS.add(gamepad1.right_stick_x);
            YAVGPos_RS.add(gamepad1.right_stick_y);
        }
        //Loops through the stack and outputs a list of averaged x and y stick positions
        for(int iterator = 0; iterator < XAVGPos_LS.size(); iterator++){
            AllStickAVGS[0] += (XAVGPos_LS.get(iterator)/XAVGPos_LS.size());
            AllStickAVGS[1] += (YAVGPos_LS.get(iterator)/XAVGPos_LS.size());
            AllStickAVGS[2] += (XAVGPos_RS.get(iterator)/XAVGPos_LS.size());
            AllStickAVGS[3] += (YAVGPos_RS.get(iterator)/XAVGPos_LS.size());
        }
        return (AllStickAVGS);
    }

    public void deadZoneFinderDriveTest(){
        double LeftStickX = gamepad1.left_stick_x;
        double LeftStickY = gamepad1.left_stick_y;
        if(Math.abs(((Math.atan2(LeftStickY, LeftStickX)*180)/Math.PI)) < Constants.deadzoneRange){
            driveRobot(LeftStickX, 0, gamepad1.right_stick_x);
            telemetry.addData("X to Y angle", ((Math.atan2(LeftStickY, LeftStickX)*180)/Math.PI));
            telemetry.update();
        } else if(Math.abs(((Math.atan2(LeftStickX, LeftStickY)*180)/Math.PI)) < Constants.deadzoneRange) {
            driveRobot(0, LeftStickY, gamepad1.right_stick_x);
            telemetry.addData("Y to X angle", ((Math.atan2(LeftStickX, LeftStickY)*180)/Math.PI));
            telemetry.update();
        }else{
            driveRobot(LeftStickX, LeftStickY, gamepad2.right_stick_x);
        }
    }


    public void TeleOpDrive(){

        Float[] allStickAVGS = GamePadInputAVG();
        //Stick deadzones:
        //FilteredInput[0] = Left stick X pos
        //FilteredInput[1] = Left stick Y pos
        //FilteredInput[2] = Right stick X pos
        //FilteredInput[3] = Right stick Y pos

        //Driving / pivoting, not sure how we want it to be structured so this can be fixed later
        deadZoneFinderDriveTest();

    }

}

