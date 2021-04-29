package global;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.vuforia.EyewearUserCalibrator;

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

    public boolean isFailing = false;

    public int checksFailed = 0;



    public void init(HardwareMap hwMap){
        lr = hwMap.get(ModernRoboticsI2cRangeSensor.class, "lr");
        br = hwMap.get(ModernRoboticsI2cRangeSensor.class, "br");
    }

    public double getLeftDistance(){
        return correctDisForLeft(lr.getDistance(DistanceUnit.CM));
    }

    public double getBackDistance(){
        return correctDisForBack(br.getDistance(DistanceUnit.CM));
    }

    public double correctDisForBack(double in){
        return (in*1.07)-0.791;
    }
    public double correctDisForLeft(double in){
        return (in*1.08)-1.09;
    }

    public void update(double heading){
        theta = heading;
        leftDis = getDisFromCenterX(getLeftDistance());
        backDis = getDisFromCenterY(getBackDistance());
    }

    public double getDisFromCenterX(double sensorDis){
        double d = Geometry.lawOfCosinesC(sensorDis, Constants.ROBOT_RADIUS, Constants.CENTER_THETA);
        double phi = Geometry.lawOfSinesAngle(sensorDis, d, Constants.CENTER_THETA);
        return d * Math.sin(Math.toRadians(theta) + phi + Constants.CENTER_THETA - Constants.halfPi);
    }
    public double getDisFromCenterY(double sensorDis){
        double centThe = Constants.tfPi - Constants.CENTER_THETA;
        double d = Geometry.lawOfCosinesC(sensorDis, Constants.ROBOT_RADIUS, centThe);
        double phi = Geometry.lawOfSinesAngle(sensorDis, d, centThe);
        return d * -Math.cos(Math.toRadians(theta) + Constants.pi2 - centThe - phi);
    }


    public double[] getPos(){
        return new double[] {leftDis, backDis};
    }

    public double[] getPos(double[] oldPos){
        double[] newPos = getPos();
        boolean xAccurate = (Math.abs(newPos[0]-oldPos[0]) < Constants.POS_ACCURACY);
        boolean yAccurate = (Math.abs(newPos[1]-oldPos[1]) < Constants.POS_ACCURACY);

        isFailing = !xAccurate || !yAccurate;
        if(isFailing){
            checksFailed +=1 ;
        }
        if (xAccurate && yAccurate) {
            return newPos;
        } else if (xAccurate) {
            return new double[]{newPos[0], oldPos[1]};
        } else if (yAccurate) {
            return new double[]{oldPos[0], newPos[1]};
        }else{
            return oldPos;
        }
    }

}
