package org.firstinspires.ftc.teamcode.Functions;

//import com.fasterxml.jackson.databind.ObjectMapper;
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import java.io.File;
import java.io.IOException;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.Functions.MV.MVVariables;

import com.qualcomm.robotcore.util.ReadWriteFile;

import java.lang.Math;

public class AccelerationDetector {

    double startingRotation =0;
    double startXAccel;
    double startYAccel;
    BNO055IMU Gyro;

    /**
     * This method initialises the accelerometer.
     * @param : gyroscope
     */
    public AccelerationDetector(BNO055IMU gyro) throws IOException {
        Gyro = gyro;
        BNO055IMU.Parameters parameter = new BNO055IMU.Parameters();
        parameter.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameter.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameter.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opMode
        parameter.loggingEnabled = true;
        parameter.loggingTag = "IMU";

        //ObjectMapper objectMapper = new ObjectMapper();
        String filename = "AdafruitIMUCalibration.json";
        File file = AppUtil.getInstance().getSettingsFile(filename);
        BNO055IMU.CalibrationData calibrationData;
        //calibrationData = objectMapper.readValue( ReadWriteFile.readFile(file), BNO055IMU.CalibrationData.class);
        //gyro.writeCalibrationData(calibrationData);
        InitializeGyro();

    }

    Position initalPosition;
    Velocity initialVelocity;

    public AccelerationDetector() {
    }

    /**
     * This method stops the code to initialise the gyro.
     */
    public boolean WaitForInitialization() {
        return !Gyro.isAccelerometerCalibrated();
    }

    MVVariables.Vector3 accelData;
    MVVariables.Vector3 accelDataMaximum = new MVVariables.Vector3(0, 0, 0);
    MVVariables.Vector3 accelDataMedium = new MVVariables.Vector3(0, 0, 0);
    MVVariables.Vector3 accelDataNumber = new MVVariables.Vector3(0, 0, 0);

    /**
     * This method calibrates the gyroscope.
     * @usage : Put it in a loop in an initialization when robot is NOT MOVING.
     * @return : (String) This returns debug data.
     */
    public String Calibration() {
        accelData = new MVVariables.Vector3(ReturnX(), ReturnY(), ReturnZ());
        accelDataMaximum.x=(accelDataMaximum.x+ accelData.x);
        accelDataNumber.x++;
        accelDataMedium.x= accelDataMaximum.x/ accelDataNumber.x;
        accelDataMaximum.y=(accelDataMaximum.y+ accelData.y);
        accelDataNumber.y++;
        accelDataMedium.y= accelDataMaximum.y/ accelDataNumber.y;
        accelDataMaximum.z=(accelDataMaximum.z+ accelData.z);
        accelDataNumber.z++;
        accelDataMedium.z= accelDataMaximum.z/ accelDataNumber.z;

        //OLD CALIBRATION
        /*
        if(AccelDataMaximum.x<AccelData.x){
            AccelDataMaximum.x=AccelData.x;
        }
        if(AccelDataMaximum.y<AccelData.y){
            AccelDataMaximum.y=AccelData.y;
        }
        if(AccelDataMaximum.z<AccelData.z){
            AccelDataMaximum.z=AccelData.z;
        }
        */

        return "AccelDataMaximum: "+ accelDataMaximum.ReturnData() + "/nAccelData: "+ accelData.ReturnData()
                + "/nAccelDataNumber: "+ accelDataNumber.ReturnData()+ "/AccelDataMedium: "+ accelDataMedium.ReturnData();
    }

    /**
     * This method initialises the gyroscope.
     */
    public void InitializeGyro() {
        startingRotation =ReturnPositiveRotation();
        startXAccel =Gyro.getLinearAcceleration().zAccel;
        startYAccel =Gyro.getLinearAcceleration().yAccel;
        initalPosition = new Position();
        initalPosition.x=0;
        initalPosition.y=0;
        initalPosition.z=0;
        initialVelocity = new Velocity();
        initialVelocity.xVeloc=0;
        initialVelocity.yVeloc=0;
        initialVelocity.zVeloc=0;
    }

    /**
     * This method returns the gyro so we don't have to reinitialise it for other clases.
     * @return : This returns gyro.
     */
    public BNO055IMU ReturnGyro() {
        return Gyro;
    }

    /**
     * This method calculates the distance travelled by the robot, on a local plan.
     * @return : (double) This returns the distance.
     */
    public double ReturnDistance() {
        if(Double.isNaN(Math.sqrt(ReturnX()*ReturnX()+ReturnY()*ReturnY()))){
            return 0;
        }
        return Math.sqrt(ReturnX()*ReturnX()+ReturnY()*ReturnY());
    }

    /**
     * This method returns the current position as vector2 (vector starting from 0,0 (origin) and ending in x,y (end))
     * @return : This returns the current position.
     */
    public MVVariables.Vector2 Position2() {
        return new MVVariables.Vector2(ReturnX(), ReturnY());
    }

    public MVVariables.Vector3 rawAccel = new MVVariables.Vector3(0, 0, 0);
    public MVVariables.Vector3 realAccel = new MVVariables.Vector3(0, 0, 0);

    /**
     * This method updates the acceleration by removing zecimals, if negative values appear they reset to 0.
     */
    public void UpdateAccel(){
        rawAccel.x = DecimalElimination(ReturnX(), 4);//-AccelDataNumber.x;
        rawAccel.y = DecimalElimination(ReturnX(), 4);//-AccelDataNumber.y;
        rawAccel.z = DecimalElimination(ReturnX(), 4);//-AccelDataNumber.z;;

        realAccel.AddVector(rawAccel);
        realAccel.ResetNegativeValues();
        //RealAccel.ResetNegativeValues();
    }

    /**
     * This methods return data of the integrated accelerometer from the gyro.
     */
    public double ReturnX() {
        return -Gyro.getLinearAcceleration().yAccel;
    }
    public double ReturnY() {
        return Gyro.getLinearAcceleration().zAccel;
    }
    public double ReturnZ() {
        return Gyro.getLinearAcceleration().xAccel;
    }

    /**
     * @return : (String) This returns the status of the gyroscope for debugging.
     */
    public String AccelerometerStatus(){
        return "The current status of the gyroscope is: " + Gyro.getCalibrationStatus();
    }

    /**
     * @return : (double) This returns the angle from 180 to -180 degrees.
     */
    double ReturnRotation(){
        return Gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
    }

    /**
     * @return : (double) This returns the angle from 0 to 360 degrees.
     */
    double ReturnPositiveRotation(){
        double currentRotation = ReturnRotation();
        if(currentRotation >= 0)
        {
            return currentRotation;
        }
        else
        {
            return 360+currentRotation;
        }

    }

    /**
     * This method corrects the angle given, if it's above 360.
     * @return : (int) This returns the corrected angle.
     */
    int AngleCorrection(int angle){
        int correctedAngle = angle;
        while(correctedAngle>=360){
            correctedAngle=correctedAngle-360;
        }
        return correctedAngle;
    }

    /**
     * This method eliminates decimals after x.
     * @return : (double) This returns new number without decimals.
     */
    double DecimalElimination(double finalNumber, int x){
        for(int i=1; i<=x;i++){
            finalNumber=finalNumber*10.0;
        }
        finalNumber = (int) finalNumber;
        for(int i=1; i<=x;i++){
            finalNumber=finalNumber/10.0;
        }
        return finalNumber;
    }

    /**
     * OLD FUNCTIONS - for backup
     */

    public void StartAcceleration(){
        Gyro.startAccelerationIntegration(initalPosition, initialVelocity, 250);
    }

    public void StopAcceleration(){
        Gyro.stopAccelerationIntegration();
    }

    public double ReturnTestX(){
        return Gyro.getPosition().x;
    }

    public double ReturnTestY(){
        return Gyro.getPosition().y;
    }

    public double ReturnTestZ(){
        return Gyro.getPosition().z;
    }
}
