package org.firstinspires.ftc.teamcode.Functions;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.Functions.MV.MVVariables;
import org.firstinspires.ftc.teamcode.Functions.MV.MVPIDController;

public class RotationDetector {
    /**
     * This class is initialised this way: rotationDetector = new RotationDetector(hardwareMap.get(BNO055IMUImpl.class, "imu"));
     * USAGE:
     * int angle = <-- here comes the angle
     * while(rotationDetector.WaitForRotation(angle)){
     *     rotate.RotateRaw(1, rotationDetector.MotorPower(angle));
     * }
     * rotate.MoveStop(); - for safety
     */

    double startingRotation = 0;
    BNO055IMU Gyro;
    MVPIDController MotorPID;
    //public Telemetry telemetry;

    public RotationDetector(BNO055IMU gyro){
        if (gyro == null) {
            throw new NullPointerException("Error: Gyro object is null");
        }
        try {
            Gyro =gyro;
            BNO055IMU.Parameters par = new BNO055IMU.Parameters();
            par.angleUnit = BNO055IMU.AngleUnit.DEGREES;
            par.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
            Gyro.initialize(par);
            while(!Gyro.isGyroCalibrated()){
                // Wait for the Gyro sensor to be calibrated
            }
            startingRotation =ReturnPositiveRotation();

            MotorPID = new MVPIDController();
            MotorPID.pidController(0.025, 0.01, 0.25, 0.01, 0.01);
            MotorPID.setContinuous(true);
            MotorPID.setInputRange(-180.0,180.0);
            MotorPID.setOutputRange(-1.0,1.0);
        } catch (Exception e) {
            System.out.println("Exception caught in setting RotationDetector: " + e.getMessage());
        }

    }

    /**
     * @return : This returns the gyro.
     */
    public BNO055IMU ReturnGyro(){
        return Gyro;
    }


    /**
     * @return : (double) This returns starting rotation.
     */
    public double ReturnStartingRotation(){
        return startingRotation;
    }

    /**
     * @return : (double) This returns the current rotation.
     */
    public double ReturnRotation(){
        return Gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
    }

    /**
     * @return : (double) This returns the POSITIVE current rotation.
     */
    public double ReturnPositiveRotation(){
        double currentRotation=ReturnRotation();
        if(currentRotation >= 0.0){
            if (currentRotation > 180.0){
                currentRotation -= 180.0;
            }
            return currentRotation;
        }
        else
        {
            return 360+currentRotation;
        }

    }

    /**
     * This method corrects an angle that's above 360 degrees.
     * @param targetRotation : (int) given angle
     * @return : true if the targetRotation has been reached, otherwise false
     */
    public boolean WaitForRotation(double targetRotation){
        if(AngleCorrection(targetRotation) == AngleCorrection((int)ReturnPositiveRotation()))
        {
            return false;
        }
        if(targetRotation-1<=(int)ReturnPositiveRotation()
                && targetRotation+1>=(int)ReturnPositiveRotation())
        {
            return false;
        }
        return true;
    }


    /**
     * This method calculates the direction simulating the robot rotation both directions and seeing
     * which one is faster.
     * @param targetRotation
     * @return (int) [-1, 1], the faster direction
     */
    int DirectionCalculator(int targetRotation) {
        int currentAngle= (int) ReturnPositiveRotation();
        MVVariables.Vector2 directions = new MVVariables.Vector2(currentAngle, currentAngle);
        for(int index=1;index<=180;index++){
            directions= new MVVariables.Vector2(AngleCorrection(currentAngle+index), AngleCorrection(currentAngle-index));
            if(directions.x==targetRotation){
                return 1;
            }
            else if(directions.y==targetRotation){
                return -1;
            }
        }
        return 1;
    }

    /*public double MotorPower(int targetRotation){
        //calculam distanta
        int modifier= DirectionCalculator(targetRotation);
        double firstDistance = Math.abs(targetRotation-(int)ReturnPositiveRotation());
        double secondDistance = 360 - firstDistance;
        // luam doar distanta cea mai mica;
        double finalDistance=0;
        if(firstDistance >= secondDistance){
            finalDistance = secondDistance;
        }
        else{
            finalDistance = firstDistance;
        }
        finalDistance = Math.abs(finalDistance);
        // inainte de calcula viteza finala trebuie sa impunem minimul distantei pentru impartire pentru
        // cazul in care distanta e prea mica si trebuie mers incet
        double deltaPower = finalDistance;
        if(finalDistance<=45 && finalDistance>10){
            return 0.2*modifier;
        }
        else if(finalDistance<=10 && finalDistance>0){
            return 0.09*modifier;
        }
        else if(finalDistance==0){
            return 0;
        }
        else
        {
            return 0.4*modifier;
        }
    }*/



    /**
     * This method makes sure that the robot slowly stops when it reaches its desire angle
     * @param targetRotation
     * * @return (double) [-1, 1] dcmotor power
     */
    public double MotorPower(double targetRotation){
        double power = MotorPID.calculate(targetRotation);
        //    double newRotationAngle = 0;
        //    try {
        //        newRotationAngle = targetRotation;
        //    }
        //   catch (Exception e) {
        //           telemetry.addData("Exception caught in MotorPower(RotationDetector): ", e.getMessage());
        //    }
        //            telemetry.update();
        //            //return 180;
        //    }
        ////try{
            //telemetry.addData("  - targetRotation value is: ", targetRotation);
            //telemetry.addData("  - power is:", power);
            //telemetry.update();
            return power;

        //}
       //atch (NullPointerException e) {
       //   //telemetry.addData("NullPointerException caught in MotorPower: " + e.getMessage());
       //   System.out.println("NullPointerException caught in MotorPower: " + e.getMessage());
      //    return 90;
       //
       // catch (Exception e) {
        //    System.out.println("Exception caught in MotorPower: " + e.getMessage());
        //    return 180;
        //}

    }

    /**
     * This method corrects an angle that's above 360 degrees.
     * @param angle : (int) given angle
     * @return : (int) corrected angle
     */
    public double AngleCorrection(double angle){
        double auxAngle=angle;
        while(auxAngle < 0.0){
            auxAngle = auxAngle + 360.0;
        }
        while(auxAngle >= 360.0){
            auxAngle = auxAngle - 360.0;
        }
        return auxAngle;
    }

    int lastAngleReported =0;

    /**
     * @return : (boolean) This returns true while the robot is rotating.
     */
    public boolean IsRotating(){
        if(lastAngleReported !=(int)ReturnRotation()){
            lastAngleReported =(int)ReturnRotation();
            return true;
        }
        return false;
    }
}
