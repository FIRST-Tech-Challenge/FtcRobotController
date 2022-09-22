package org.firstinspires.ftc.teamcode.ebotsenums;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.ultimategoal2020.EncoderTracker;
import org.firstinspires.ftc.teamcode.ultimategoal2020.WheelPosition2020;

import java.util.ArrayList;
import java.util.HashMap;

public enum EncoderConfig {
    FOR_2019(RobotDesign.SEASON_2019),
    FOR_2020(RobotDesign.SEASON_2020);

    private DcMotorEx motor;
    private RobotOrientation robotOrientation;
    private EncoderTracker.SpinBehavior spinBehavior;          //
    private EncoderTracker.ClickDirection clickDirection;
    private EncoderModel encoderModel;
    private ArrayList<EncoderTracker> encoders;


    private static final HashMap<String, WheelPosition2020> ENC_POS_2019 = new HashMap<String, WheelPosition2020>(){
        {
            put("Forward", WheelPosition2020.BACK_RIGHT);
            put("Lateral", WheelPosition2020.FRONT_RIGHT);
            put("Third", WheelPosition2020.FRONT_LEFT);

        }
    };

    private static final HashMap<String, WheelPosition2020> ENC_POS_2020 = new HashMap<String, WheelPosition2020>(){
        {
            put("Forward", WheelPosition2020.FRONT_LEFT);
            put("Lateral", WheelPosition2020.BACK_LEFT);
            put("Third", WheelPosition2020.FRONT_LEFT);      //there are only 2 on this robot, so repeat

        }
    };



    EncoderConfig(RobotDesign robotDesign){

        // Create the first forward encoder

        
    }
}
