package org.firstinspires.ftc.teamcode.quantumstorm.Sensors;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.quantumstorm.Constants;

/** Optional four-slot presence/color sensing. Missing or uncalibrated hardware stays UNKNOWN. */
public final class PollenInventory {
    public static final class Sample {
        public final int total,pollen;
        Sample(int total,int pollen){this.total=total;this.pollen=pollen;}
    }
    private final NormalizedColorSensor[] colors=new NormalizedColorSensor[4];
    private final DistanceSensor[] distances=new DistanceSensor[4];
    private boolean ready;
    public PollenInventory(HardwareMap map){
        String[] names=Constants.TEAM_TWO_POLLEN_SLOT_SENSOR_NAMES;
        ready=Constants.TEAM_TWO_INVENTORY_CALIBRATED&&names.length==4;
        if(!ready)return;
        for(int i=0;i<4;i++){
            checkStop();colors[i]=map.tryGet(NormalizedColorSensor.class,names[i]);distances[i]=map.tryGet(DistanceSensor.class,names[i]);
            if(colors[i]==null||distances[i]==null)ready=false;
        }
    }
    private void checkStop(){if(Thread.currentThread().isInterrupted())throw new java.util.concurrent.CancellationException("Inventory read cancelled");}
    public Sample read(){
        if(!ready)return new Sample(-1,-1);
        try{
            int total=0,pollen=0;boolean typed=true;
            for(int i=0;i<4;i++){
                checkStop();double cm=distances[i].getDistance(DistanceUnit.CM);checkStop();
                if(!Double.isFinite(cm)||cm<=0)return new Sample(-1,-1);
                if(cm>Constants.TEAM_TWO_SLOT_OCCUPIED_MAX_CM)continue;
                total++;NormalizedRGBA c=colors[i].getNormalizedColors();checkStop();
                if(c==null||!Float.isFinite(c.red)||!Float.isFinite(c.green)||!Float.isFinite(c.blue)||!Float.isFinite(c.alpha)
                        ||c.alpha<Constants.TEAM_TWO_SLOT_MIN_ALPHA){typed=false;continue;}
                if(isYellow(c.red,c.green,c.blue))pollen++;
            }
            return new Sample(total,typed?pollen:-1);
        }catch(java.util.concurrent.CancellationException e){throw e;}
        catch(RuntimeException e){return new Sample(-1,-1);}
    }
    public static boolean isYellow(double red,double green,double blue){
        if(!Double.isFinite(red)||!Double.isFinite(green)||!Double.isFinite(blue)||red<=0||green<=0||blue<0)return false;
        return red>blue*Constants.TEAM_TWO_SLOT_YELLOW_BLUE_RATIO&&green>blue*Constants.TEAM_TWO_SLOT_YELLOW_BLUE_RATIO
                &&red/green>=Constants.TEAM_TWO_SLOT_YELLOW_RG_MIN&&red/green<=Constants.TEAM_TWO_SLOT_YELLOW_RG_MAX;
    }
    public String status(){return ready?"Four calibrated presence/color slots":"Inventory UNKNOWN: configure/calibrate four slot sensors; Team Two shooting disabled";}
}
