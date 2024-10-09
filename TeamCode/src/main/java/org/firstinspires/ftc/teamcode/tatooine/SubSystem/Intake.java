package org.firstinspires.ftc.teamcode.tatooine.SubSystem;


import android.graphics.ColorSpace;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.teamcode.tatooine.utils.SampleColor;
import org.opencv.core.Scalar;

public class Intake  {
    CRServo intake = null;
    NormalizedColorSensor normalizedColorSensor = null;
    SampleColor sampleColor;

    private double power = 0;
    private boolean buttonPressed = false;

    public Intake(HardwareMap hardwareMap, SampleColor sampleColor) {
        this.sampleColor = sampleColor;
        intake = hardwareMap.get(CRServo.class, "intake");
        normalizedColorSensor = hardwareMap.get(NormalizedColorSensor.class, "Color Sensor");
    }
    public void chooseColor(){
        if (sampleColor == SampleColor.BLUE || sampleColor == SampleColor.BLUEANDYELLOW){
            
        }
        else{

        }
    }
    public Action intake(boolean buttonPressed){
        this.buttonPressed = buttonPressed;
        power = 1;
        return new SetPowerAction();
    }
    public Action outtake(){
        power = -1;
        return new SetPowerAction();
    }

    public class SetPowerAction implements Action{
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            intake.setPower(power);
            return buttonPressed;
        }
    }
}