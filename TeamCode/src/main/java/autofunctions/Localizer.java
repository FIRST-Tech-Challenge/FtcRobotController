package autofunctions;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import globalfunctions.Constants;
import util.CodeSeg;
import util.Geometry;
import util.Vector;

public class Localizer {

    public double leftDis = 0;
    public double backDis = 0;
    public double theta = 0;

    public ModernRoboticsI2cRangeSensor lr;
    public ModernRoboticsI2cRangeSensor br;



    public Geometry geometry = new Geometry();

    public void init(HardwareMap hwMap){
        lr = hwMap.get(ModernRoboticsI2cRangeSensor.class, "lr");
        br = hwMap.get(ModernRoboticsI2cRangeSensor.class, "br");
    }

    public double getLeftDistance(){
        return lr.getDistance(DistanceUnit.CM);
    }

    public double getBackDistance(){
        return br.getDistance(DistanceUnit.CM);
    }

    public void update(double heading){
        theta = heading;
        leftDis = getDisFromCenter(getLeftDistance());
        backDis = getDisFromCenter(getBackDistance());
    }

    public double getDisFromCenter(double sensorDis){
        double d = geometry.lawOfCosinesC(sensorDis, Constants.ROBOT_RADIUS, Constants.CENTER_THETA);
        double phi = geometry.lawOfSinesAngle(Constants.ROBOT_RADIUS, d, Constants.CENTER_THETA);
        return d * Math.cos(Math.toRadians(theta)+ phi);
    }

    public double[] getPos(double heading){
        update(heading);
        double[] out = new double[2];
        out[0] = leftDis;
        out[1] = backDis;
        return out;
    }

}
