package org.firstinspires.ftc.teamcode.Functions;

import com.qualcomm.hardware.bosch.BNO055IMU;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

//import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.Functions.MV.MVVariables;
import org.firstinspires.ftc.teamcode.Functions.MV.MVPIDController;

/**
 * This is the constructor for the RotationDetector class. It takes in a BNO055IMU object,
 * which is a type of gyro sensor, and initializes it for use. It sets the angle unit of the
 * gyro to degrees and the acceleration unit to meters per second squared. It also initializes
 * a PID controller, which will be used to calculate the motor power needed to rotate the robot
 * to a specific angle.
 * <p></p>
 * This class is initialised this way:
 * <pre>{@code
 * rotationDetector = new RotationDetector(hardwareMap.get(BNO055IMUImpl.class, "imu"));
 * }</pre>
 * USAGE:
 * <pre>{@code
 *   int angle = // <-- here comes the angle
 *      while(rotationDetector.WaitForRotation(angle)){
 *          rotate.RotateRaw(1, rotationDetector.MotorPower(angle));
 *      }
 *      rotate.MoveStop(); // for safety
 * }</pre>
 */
public class RotationDetector {

    double startingRotation = 0.0;
    double PIDOutput = 0.0;
    BNO055IMU Gyro;
    MVPIDController MotorPID;
    //public Telemetry telemetry;

    public RotationDetector(BNO055IMU gyro){
        if (gyro == null) {
            throw new NullPointerException("Error: Gyro object is null");
        }
        try {
            Gyro = gyro;
            BNO055IMU.Parameters par = new BNO055IMU.Parameters();
            par.angleUnit = BNO055IMU.AngleUnit.DEGREES;
            par.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
            Gyro.initialize(par);
            while(!Gyro.isGyroCalibrated()){
                // Wait for the Gyro sensor to be calibrated
            }
            startingRotation = ReturnRotation();

            MotorPID = new MVPIDController();
            MotorPID.pidController(0.01, 0.005, 0.025, 0.1, 0.001);
            MotorPID.setContinuous(true);
            MotorPID.setInputRange(Math.toRadians(-180.0),Math.toRadians(180.0));
            MotorPID.setOutputRange(-1.0,1.0);
        } catch (Exception e) {
            System.out.println("Exception caught in setting RotationDetector: " + e.getMessage());
        }

    }

    /** This method simply returns the BNO055IMU Gyro object.
     * @return : This returns the gyro.
     */
    public BNO055IMU ReturnGyro(){
        return Gyro;
    }

    /** This method simply returns the PID Controller object.
     * @return : This returns the PID.
     */
    public MVPIDController ReturnPIDController(){
        return MotorPID;
    }

    /**
     * @return : (double) This returns starting rotation.
     */
    public double getStartingRotationDeg(){
        return Math.toDegrees(startingRotation);
    }

    /**
     * @return : (double) This returns starting rotation.
     */
    public double getStartingRotationRad(){
        return startingRotation;
    }

    /**
     * This method sets the starting rotation angle of the robot to the current rotation, which
     * can be useful if you want to rotate the robot relative to its current position rather
     * than its starting position (0 degrees).
     */
    public void setStartingRotationToCurrent() {
        // Set the starting rotation to the current rotation
        startingRotation = ReturnRotation();
    }

    /**
     * This method sets the starting rotation angle of the robot to the given angle.
     *
     * @param angle The angle in degrees (-180 to 180).
     */
    public void setStartingRotationToAngle(double angle) {
        // Normalize the angle to be between -180 and 180 degrees
        angle = AngleUnit.DEGREES.normalize(angle);

        // Set the starting rotation to the given angle
        startingRotation = angle;
    }

    /**
     * This method sets the starting rotation angle of the robot to the current rotation, which
     * can be useful if you want to rotate the robot relative to its current position rather
     * than its starting position (0 degrees).
     */
    public void resetStartingRotation() {
        // Reset the starting rotation to the default direction (o degrees)
        startingRotation = 0.0;
    }

    /**
     * This method gets the current orientation of the IMU sensor
     * @return : (double[3]) This returns the current orientation vector (angles).
     */
    public double[] getOrientation(){
        double[] orientation = new double[3];
        orientation[0] = Gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        orientation[1] = Gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).secondAngle;
        orientation[2] = Gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).thirdAngle;
        return orientation;
    }

    /**
     * This method gets the current PID Controller parameters (if a PID Controller is used)/
     * @return : (double[5]) This returns the P, I, D, F and tolerance values.
     */
    public double[] getPIDParameters(){
        double[] parameters = new double[] {0, 0, 0, 0, 0};
        if (MotorPID != null) {
            // MotorPID is not null
            parameters = MotorPID.getPIDParameters();
        }
        return parameters;
    }

    /**
     * This method returns the current rotation of the robot as reported by the gyro sensor.
     * <p></p>
     * It does this by using the getAngularOrientation method of the BNO055IMU class to get the
     * orientation of the robot in degrees and returning the first angle, which corresponds to
     * the rotation around the Z axis.
     * @return : (double) This returns the current rotation.
     */
    public double ReturnRotation(){
        return Gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
    }

    /**
     * This method returns the current rotation of the robot as reported by the gyro sensor.
     * <p></p>
     * It does this by using the getAngularOrientation method of the BNO055IMU class to get the
     * orientation of the robot in degrees and returning the first angle, which corresponds to
     * the rotation around the Z axis.
     * @return : (double) This returns the current rotation.
     */
    public double ReturnRotationRad(){
        return Gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle;
    }

    /**
     * @return : (double) This returns the POSITIVE current rotation.
     */
    public double ReturnPositiveRotation(){
        double currentRotation=ReturnRotation();
        if(currentRotation>=0)
        {
            return currentRotation;
        }
        else
        {
            return 360+currentRotation;
        }

    }

    /**
     * This method waits until the robot has rotated to the specified angle. It does this by
     * continuously getting the current rotation of the robot and comparing it to the target
     * rotation until they are within a certain tolerance of each other.
     * <p></p>
     * It returns true if the robot has not yet reached the target angle and false if it has.
     * @param targetRotationRad : (double) given angle in Radians
     * @return : true if the targetRotation has been reached, otherwise false
     */
    public boolean WaitForRotation(double targetRotationRad){
        double targetAngle = AngleUnit.RADIANS.normalize(targetRotationRad); // Normalize the targeted angle
        double currentAngle = ReturnRotationRad();  // Get the current angle of the robot in radians

        //targetAngle = MotorPID.wrapToRange(targetAngle);
        //if (MotorPID.onTarget()) {
        //    return false;
        //}
        MotorPID.setStart(currentAngle); // Set the starting angle for the PID controller
        MotorPID.setTarget(targetAngle); // Set the target angle for the PID controller

        // Calculate the output of the PID controller
        double output = MotorPID.calculate();
        PIDOutput = output;

        // If the error is within the tolerance
        if (Math.abs(output) > MotorPID.getTolerance()) {
                return true;  // Return true, indicating that the target angle has not been reached
        } else {
            return false;  // Return false, indicating that the target angle has been reached
        }

        //if(AngleCorrection(targetRotation) == AngleCorrection((int)ReturnPositiveRotation()))
        //{
        //    return false;
        //}
        //if(targetRotation-1<=(int)ReturnPositiveRotation()
        //        && targetRotation+1>=(int)ReturnPositiveRotation())
        //{
        //    return false;
        //}
    }

    /** Method to reset the PIDOutput property.
     * @return : (double) This returns the POSITIVE current rotation.
     */
    public void resetPIDOutput(){
        PIDOutput = 0.0;
    }

    /** Method to get the PIDOutput property.
     * @return : (double) This returns the POSITIVE current rotation.
     */
    public double getPIDOutput(){
        return PIDOutput;
    }

    /**
     * This method calculates the direction simulating the robot rotation both directions and seeing
     * which one is faster.
     * @param targetRotation
     * @return (int) [-1, 1], the faster direction
     */
     //int DirectionCalculator(int targetRotation) {
     //   int currentAngle= (int) ReturnPositiveRotation();
     //   MVVariables.Vector2 directions = new MVVariables.Vector2(currentAngle, currentAngle);
     //   for(int index=1;index<=180;index++){
     //       directions= new MVVariables.Vector2(AngleCorrection(currentAngle+index), AngleCorrection(currentAngle-index));
     //       if(directions.x==targetRotation){
     //           return 1;
     //       }
     //       else if(directions.y==targetRotation){
     //           return -1;
     //       }
     //   }
     //   return 1;
    //}


    /**
     * This method calculates the motor power needed to rotate the robot to the specified angle
     * using a PID controller. It does this by first setting the target value of the PID controller
     * to the target rotation and then using the calculate() method of the MVPIDController class
     * to calculate the motor power based on the current rotation of the robot.
     * <p></p>
     * The MVPIDController class is a custom PID controller implementation.
     * <p></p>
     * The motor power is returned as a value between -1.0 and 1.0, where negative values indicate
     * rotation in the negative direction and positive values indicate rotation in the positive
     * direction.
     * <p></p>
     * This method makes sure that the robot slowly stops when it reaches its desire angle
     * @param targetRotation
     * * @return (double) [-1, 1] dcmotor power
     */
    public double MotorPower(double targetRotation){
        MotorPID.setTarget(targetRotation);
        double power = MotorPID.calculate();
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
    //public double AngleCorrection(double angle){
    //    double auxAngle=angle;
    //    while(auxAngle < 0.0){
    //        auxAngle = auxAngle + 360.0;
    //    }
    //    while(auxAngle >= 360.0){
    //        auxAngle = auxAngle - 360.0;
    //    }
    //    return auxAngle;
    //}

    int lastAngleReported = 0;

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
