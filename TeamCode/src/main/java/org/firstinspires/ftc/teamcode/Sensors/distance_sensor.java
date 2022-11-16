package org.firstinspires.ftc.teamcode.Sensors;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class distance_sensor {
    /*
    private DistanceSensor front;
    private DistanceSensor back;
    private DistanceSensor left;
    private DistanceSensor right;

    public distance_sensor(HardwareMap hwMap){
        this.front = front;
        this.back = back;
        this.left = left;
        this.right = right;
    }

    public double getDistance(String Sensor){
        if(Sensor.equals("front")){
            return front.getDistance(DistanceUnit.METER);
        }
        else if(Sensor.equals("back")){
            return back.getDistance(DistanceUnit.METER);
        }
        else if(Sensor.equals("left")){
            return left.getDistance(DistanceUnit.METER);
        }
        else if(Sensor.equals("right")){
            return right.getDistance(DistanceUnit.METER);
        }

        return -1;
    }
    */
    Telemetry telemetry;
    public DistanceSensor dist;

    public distance_sensor( HardwareMap hardwareMap, String distSensorName, Telemetry telemetry ) {
        this.telemetry = telemetry;
        setup( hardwareMap, distSensorName );
    }

    public void setup( HardwareMap hardwareMap, String distSensorName ) {
        dist = hardwareMap.get(DistanceSensor.class,distSensorName);
    }

    public double getDistance(){
        return dist.getDistance(DistanceUnit.CM);
    }
}
