package org.firstinspires.ftc.teamcode;
/*
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class FindStone {

    ColorSensor rfColorSensor;
    ColorSensor rbColorSensor;
    ColorSensor lfColorSensor;
    ColorSensor lbColorSensor;

    int red;
    int green;
    int blue;

    int rfred;
    int rfgreen;
    int rfblue;

    int rbred;
    int rbgreen;
    int rbblue;

    int lfred;
    int lfgreen;
    int lfblue;

    int lbred;
    int lbgreen;
    int lbblue;

    HardwareMap hwMap           = null;

    public FindStone(){

    }

    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        rfColorSensor = hwMap.get(ColorSensor.class,"rfColor");
        rbColorSensor = hwMap.get(ColorSensor.class,"rbColor");
        lfColorSensor = hwMap.get(ColorSensor.class,"lfColor");
        lbColorSensor = hwMap.get(ColorSensor.class,"lbColor");

        rfColorSensor.enableLed(false);
        rbColorSensor.enableLed(false);
        lfColorSensor.enableLed(false);
        lbColorSensor.enableLed(false);
    }

    public void rfColorLED(boolean onOff){
        if (onOff){rfColorSensor.enableLed(true);
        }else if (!onOff){rfColorSensor.enableLed(false);}
    }
    public void rbColorLED(boolean onOff){
        if (onOff){rbColorSensor.enableLed(true);
        }else if (!onOff){rbColorSensor.enableLed(false);}
    }
    public void lfColorLED(boolean onOff){
        if (onOff){lfColorSensor.enableLed(true);
        }else if (!onOff){lfColorSensor.enableLed(false);}
    }
    public void lbColorLED(boolean onOff){
        if (onOff){lbColorSensor.enableLed(true);
        }else if (!onOff){lbColorSensor.enableLed(false);}
    }
    /*public StoneID isSkyStone(StoneID sensorSide) {
        if (sensorSide == StoneID.LEFT){
            lColorSensor.enableLed(true);
            red   = lColorSensor.red();
            green = lColorSensor.green();
            blue  = lColorSensor.blue();
        }else if (sensorSide == StoneID.RIGHT){
            rColorSensor.enableLed(true);
            red   = rColorSensor.red();
            green = rColorSensor.green();
            blue  = rColorSensor.blue();
        }

        if (red < 10 && green < 10) {
            lColorSensor.enableLed(false);
            rColorSensor.enableLed(false);
            return StoneID.SKYSTONE;
        }else if (red > 12 && green > 12) {
            lColorSensor.enableLed(false);
            rColorSensor.enableLed(false);
            return StoneID.STONE;
        }else {
            lColorSensor.enableLed(false);
            rColorSensor.enableLed(false);
            return StoneID.UNKNOWN;
        }
    }*/
    /*public StoneID rightSkystoneTest() {
        rfColorSensor.enableLed(true);
        rbColorSensor.enableLed(true);
        rfgreen = rfColorSensor.green();
        rbgreen = rbColorSensor.green();

        if (Math.abs(rfgreen - rbgreen) > 50){
            if (rfgreen < rbgreen){
                return StoneID.ONE;
            }else if (rfgreen > rbgreen){
                return StoneID.TWO;
            }
        }else if (Math.abs(rfgreen - rbgreen) < 50){
            return StoneID.THREE;
        }
        return StoneID.UNKNOWN;
    }
    public StoneID leftSkystoneTest() {
        lfColorSensor.enableLed(true);
        lbColorSensor.enableLed(true);
        lfgreen = lfColorSensor.green();
        lbgreen = lbColorSensor.green();

        if (Math.abs(lfgreen - lbgreen) > 50){
            if (lfgreen < lbgreen){
                return StoneID.ONE;
            }else if (lfgreen > lbgreen){
                return StoneID.TWO;
            }
        }else if (Math.abs(lfgreen - lbgreen) < 50){
            return StoneID.THREE;
        }
        return StoneID.UNKNOWN;
    }
    public int rfGetRedVal(){
        return rfColorSensor.red();
    }
    public int rfGetGreenVal(){
        return rfColorSensor.green();
    }
    public int rfGetBlueVal(){
        return rfColorSensor.blue();
    }
    public int rbGetRedVal(){
        return rbColorSensor.red();
    }
    public int rbGetGreenVal(){
        return rbColorSensor.green();
    }
    public int rbGetBlueVal(){
        return rbColorSensor.blue();
    }
}
*/