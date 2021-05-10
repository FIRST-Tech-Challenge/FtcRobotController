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
    //The distance from the center of the robot to the left wall
    public double leftDis = 0;
    //The distance from the center of the robot to the back wall
    public double backDis = 0;
    //The angle of the robot
    public double theta = 0;
    //The left distance sensor
    public ModernRoboticsI2cRangeSensor lr;
    //The back distance sensor
    public ModernRoboticsI2cRangeSensor br;
    //Is the localizer failing
    public boolean isFailing = false;
    //How many chekcs have failed
    public int checksFailed = 0;


    //Initializes the distance sensors
    public void init(HardwareMap hwMap){
        lr = hwMap.get(ModernRoboticsI2cRangeSensor.class, "lr");
        br = hwMap.get(ModernRoboticsI2cRangeSensor.class, "br");
    }
    //Gets the left distance in cm
    public double getLeftDistance(){
        return correctDisForLeft(lr.getDistance(DistanceUnit.CM));
    }
    //Gets the back distance in cm
    public double getBackDistance(){
        return correctDisForBack(br.getDistance(DistanceUnit.CM));
    }
    //Corrects the back distance sensor readings
    public double correctDisForBack(double in){
        return (in*1.07)-0.791;
    }
    //Corrects the left distance sensor readings
    public double correctDisForLeft(double in){
        return (in*1.08)-1.09;
    }
    //Updates the distance sensors values
    public void update(double heading){
        theta = heading;
        leftDis = getDisFromCenterX(getLeftDistance());
        backDis = getDisFromCenterY(getBackDistance());
    }
    //Gets the distance from the center of the robot in the x direction
    public double getDisFromCenterX(double sensorDis){
        double d = Geometry.lawOfCosinesC(sensorDis, Constants.ROBOT_RADIUS, Constants.CENTER_THETA);
        double phi = Geometry.lawOfSinesAngle(sensorDis, d, Constants.CENTER_THETA);
        return d * Math.sin(Math.toRadians(theta) + phi + Constants.CENTER_THETA - Constants.halfPi);
    }
    //Gets the distance from the center of the robot in the y direction
    public double getDisFromCenterY(double sensorDis){
        double centThe = Constants.tfPi - Constants.CENTER_THETA;
        double d = Geometry.lawOfCosinesC(sensorDis, Constants.ROBOT_RADIUS, centThe);
        double phi = Geometry.lawOfSinesAngle(sensorDis, d, centThe);
        return d * -Math.cos(Math.toRadians(theta) + Constants.pi2 - centThe - phi);
    }
    //Gets the position of the robot
    public double[] getPos(){
        return new double[] {leftDis, backDis};
    }
    //Gets the position of the robot and checks if the position is close to the last position
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
