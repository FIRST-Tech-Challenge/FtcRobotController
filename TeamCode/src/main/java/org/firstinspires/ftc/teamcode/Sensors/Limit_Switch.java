package org.firstinspires.ftc.teamcode.Sensors;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Limit_Switch {
    Telemetry telemetry;
    TouchSensor limit1;

    public Limit_Switch(HardwareMap hardwareMap, String limitSwitchName, Telemetry telemetry ) {
        this.telemetry = telemetry;
        setup( hardwareMap, limitSwitchName);
    }

    public void setup( HardwareMap hardwareMap, String limitSwitchName ) {
        limit1 = hardwareMap.get(TouchSensor.class, limitSwitchName );
    }

   public boolean isPressed(){
        return limit1.isPressed();
   }
}
