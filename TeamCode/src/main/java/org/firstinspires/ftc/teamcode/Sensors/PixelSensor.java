package org.firstinspires.ftc.teamcode.hardware.Sensors;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

public class PixelSensor {

    ColorSensor sensor;

    String color;

    ElapsedTime timer = new ElapsedTime();

    public PixelSensor(ColorSensor c){
        sensor = c;
        color = "NOTHING";
    }

    public String findColor(int blue, int red, int green){
        double ratiorb = (double)red/blue;
        double ratiogb = (double)green/blue;

        double tolerance = 0.2;
        if(Math.abs(ratiorb-2.672)<=tolerance && Math.abs(ratiogb-3.78)<=tolerance){
            color = "YELLOW";
            return color;
        }
        if(Math.abs(ratiorb-0.5)<=tolerance && Math.abs(ratiogb-0.7)<=tolerance){
            color = "PURPLE";
            return color;
        }
        if(Math.abs(ratiorb-0.7)<=tolerance && Math.abs(ratiogb-2.5)<=tolerance){
            color = "GREEN";
            return color;
        }
        if(Math.abs(ratiorb-0.66)<=tolerance && Math.abs(ratiogb-1.2)<=tolerance){
            color = "WHITE";
            return color;
        }
        color = "NOTHING";
        return color;
    }

    public int getRed(){
        return sensor.red();
    }

    public int getBlue(){
        return sensor.blue();
    }
    public int getGreen(){
        return sensor.green();
    }

    public String getColor(){
        double ratiorb = (double)getRed()/getBlue();
        double ratiogb = (double)getGreen()/getBlue();

        double tolerance = 0.2;
        if(Math.abs(ratiorb-2.672)<=tolerance && Math.abs(ratiogb-3.78)<=tolerance){
            color = "YELLOW";
            return color;
        }
        if(Math.abs(ratiorb-0.5)<=tolerance && Math.abs(ratiogb-0.7)<=tolerance){
            color = "PURPLE";
            return color;
        }
        if(Math.abs(ratiorb-0.7)<=tolerance && Math.abs(ratiogb-2.5)<=tolerance){
            color = "GREEN";
            return color;
        }
        if(Math.abs(ratiorb-0.66)<=tolerance && Math.abs(ratiogb-1.2)<=tolerance){
            color = "WHITE";
            return color;
        }
        color = "NOTHING";
        return color;
    }

    public boolean present(){
        boolean signal = true;
        int currentTry = 0;
        int tries = 2;
        double timeStamp = timer.milliseconds();
        while(timer.milliseconds()<timeStamp+250){
            if(!getColor().equals("NOTHING")&&currentTry<=tries){
                signal=true;
            }else{
                signal = false;
                if(currentTry<=tries){
                    currentTry++;
                }else{
                    break;
                }
            }
        }
        return signal;
    }

}