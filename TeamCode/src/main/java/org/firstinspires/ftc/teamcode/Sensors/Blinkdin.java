package org.firstinspires.ftc.teamcode.hardware.Sensors;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;

public class Blinkdin {

    RevBlinkinLedDriver.BlinkinPattern target;
    RevBlinkinLedDriver led;

    public Blinkdin(RevBlinkinLedDriver led){
        this.led = led;
    }

    public Blinkdin changePattern(RevBlinkinLedDriver.BlinkinPattern setting){
        this.target = setting;
        return this;
    }

    public Blinkdin changeColor(String color){
        if(color.equals("YELLOW")){
            target = RevBlinkinLedDriver.BlinkinPattern.YELLOW;
        }else if(color.equals("GREEN")){
            target = RevBlinkinLedDriver.BlinkinPattern.GREEN;
        }else if(color.equals("PURPLE")){
            target = RevBlinkinLedDriver.BlinkinPattern.VIOLET;
        }else if(color.equals("WHITE")){
            target = RevBlinkinLedDriver.BlinkinPattern.WHITE;
        }else{
            target = RevBlinkinLedDriver.BlinkinPattern.RED;
        }
        return this;
    }

    public RevBlinkinLedDriver.BlinkinPattern getPattern(){
        return target;
    }
    public void update(){
        this.led.setPattern(target);
    }

}
