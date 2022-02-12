package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

// For the IMU
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

@Autonomous
//@Disabled
public class RotationTest extends LinearOpMode{

    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;
    
    // Variables for the IMU 
    BNO055IMU               imu;
    Orientation             lastAngles = new Orientation();
    double                  globalAngle, power = .30, correction;

    private Servo intakeServo ;

    @Override
    public void runOpMode() {
        
        initializeRobot();
        
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        if (opModeIsActive()) {
            
            telemetry.addData("Status", "Running");
            telemetry.update();

            sleep(5000) ;
            
            testIntakeServo(1.0) ;
            //testIMU();
            
            //forward
            /*
            frontLeft.setPower(0.75);
            backLeft.setPower(0.75);
            frontRight.setPower(0.75);
            backRight.setPower(0.75);
            sleep(3000);
            
            frontLeft.setPower(0);
            backLeft.setPower(0);
            frontRight.setPower(0);
            backRight.setPower(0);
            
            sleep(5000);
            */
            
            //backward
            /*
            frontLeft.setPower(-0.5);
            backLeft.setPower(-0.5);
            frontRight.setPower(-0.5);
            backRight.setPower(-0.5);
            sleep(4000);
            
            frontLeft.setPower(0);
            backLeft.setPower(0);
            frontRight.setPower(0);
            backRight.setPower(0);
           
            sleep(5000);
            */
            
            /*
            //strafe right
            frontLeft.setPower(-0.5);
            backLeft.setPower(0.5);
            frontRight.setPower(0.5);
            backRight.setPower(-0.5);
            sleep(1000);
            
            frontLeft.setPower(0);
            backLeft.setPower(0);
            frontRight.setPower(0);
            backRight.setPower(0);
           
            sleep(5000);
            
            //strafe left
            frontLeft.setPower(0.5);
            backLeft.setPower(-0.5);
            frontRight.setPower(-0.5);
            backRight.setPower(0.5);
            sleep(1000);
            
            frontLeft.setPower(0);
            backLeft.setPower(0);
            frontRight.setPower(0);
            backRight.setPower(0);
           
            sleep(5000);
            
            //forward right
            frontLeft.setPower(0);
            backLeft.setPower(0.5);
            frontRight.setPower(0.5);
            backRight.setPower(0);
            sleep(1000);
            
            frontLeft.setPower(0);
            backLeft.setPower(0);
            frontRight.setPower(0);
            backRight.setPower(0);
            
            sleep(5000);
            
            //forward left
            frontLeft.setPower(0.5);
            backLeft.setPower(0);
            frontRight.setPower(0);
            backRight.setPower(0.5);
            sleep(1000);
            
            frontLeft.setPower(0);
            backLeft.setPower(0);
            frontRight.setPower(0);
            backRight.setPower(0);
            
            sleep(5000);
            
            //backward right
            frontLeft.setPower(-0.5);
            backLeft.setPower(0);
            frontRight.setPower(0);
            backRight.setPower(-0.5);
            sleep(1000);
            
            frontLeft.setPower(0);
            backLeft.setPower(0);
            frontRight.setPower(0);
            backRight.setPower(0);
            
            sleep(5000);
            
            //backward left
            frontLeft.setPower(0);
            backLeft.setPower(-0.5);
            frontRight.setPower(-0.5);
            backRight.setPower(0);
            sleep(1000);
            
            frontLeft.setPower(0);
            backLeft.setPower(0);
            frontRight.setPower(0);
            backRight.setPower(0);
            */
            
            
        }
    }
    
    
    private void initializeRobot() {
        
        frontLeft = hardwareMap.get(DcMotor.class,"FrontLeft");
        frontRight = hardwareMap.get(DcMotor.class,"FrontRight");
        backLeft = hardwareMap.get(DcMotor.class,"BackLeft");
        backRight = hardwareMap.get(DcMotor.class,"BackRight");
        
        frontLeft.setDirection(DcMotor.Direction.FORWARD) ;
        frontRight.setDirection(DcMotor.Direction.REVERSE) ;
        backLeft.setDirection(DcMotor.Direction.FORWARD) ;
        backRight.setDirection(DcMotor.Direction.REVERSE) ;
        
        intakeServo = hardwareMap.get(Servo.class, "cappingServo") ;

        
        initializeIMU();
        telemetry.addData("Status", "Initialized");
        telemetry.update();

    }
    
  private void initializeIMU() {
    BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

    parameters.mode                = BNO055IMU.SensorMode.IMU;
    parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
    parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
    parameters.loggingEnabled      = false;    

    // get and initialize the IMU 
    // The imu is assumed to be on I2C port 
    // and configured to be a sensor of type "AdaFruit IMU" and named "imu"
    imu = hardwareMap.get(BNO055IMU.class, "imu");

    imu.initialize(parameters);

    telemetry.addData("Mode", "IMU calibrating...");
    telemetry.update();

    // make sure the imu gyro is calibrated before continuing.
    while (!isStopRequested() && !imu.isGyroCalibrated())
    {
      sleep(50);
      idle();
    }

    telemetry.addData("imu calib status", imu.getCalibrationStatus().toString());
    telemetry.update( );
  }

  private void displayOrientationValues(Orientation orient, String name) {
    // ensure that the order is set up correctly as specified in getAngularDistance
    telemetry.addData(name, "Z: " + orient.firstAngle + ", Y: " + orient.secondAngle + ", X: " + orient.thirdAngle);
  }
  
  private void resetAngle() {
    lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

    globalAngle = 0;
  }

  private double getAngle() {
        
    Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

    double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

    if (deltaAngle < -180)
        deltaAngle += 360;
    else if (deltaAngle > 180)
        deltaAngle -= 360;

    globalAngle += deltaAngle;

    lastAngles = angles;

    return globalAngle;
  }

    
  private double checkDirection() {
        
    double correction, angle, gain = .1;

    angle = getAngle();

    if (angle == 0)
        correction = 0;             // no adjustment.
    else
        correction = -angle;        // reverse sign of angle for correction.

    correction = correction * gain;

    return correction;
  }
    
  private void rotate(int degrees, double power) {
    double  leftPower, rightPower;

    // restart imu movement tracking.
    resetAngle();

    // getAngle() returns + when rotating counter clockwise (left) and - when rotating
    // clockwise (right).

    if (degrees < 0)
    {   // turn left
        leftPower = -power;
        rightPower = power;
    }
    else if (degrees > 0)
    {   // turn right
        leftPower = power;
        rightPower = -power;
    }
    else return;

    // set power to rotate.
    frontLeft.setPower(leftPower);
    backLeft.setPower(leftPower);
    frontRight.setPower(rightPower);
    backRight.setPower(rightPower);
    
    // rotate until turn is completed.
    if (degrees < 0)
    {
        // On right turn we have to get off zero first.
        while (getAngle() == 0) {}

        while (getAngle() > degrees) {}
    }
    else    // right turn.
        while (getAngle() < degrees) {}

    // turn the motors off.
    stopAllWheelsPower();
    

    // reset angle tracking on new heading.
    resetAngle();
  }
    
    void stopAllWheelsPower() {
        frontLeft.setPower(0.0);
        backLeft.setPower(0.0);
        frontRight.setPower(0.0);
        backRight.setPower(0.0);
    }    
    
      private void testIMU() {
    // it is assumed here the the IMU has been initialized
    
    // first reset and then get the angles 
    // reset here does not mean IMU reset. just storing it inside lastAngles and 
    // making global angles as zero 

    resetAngle();
    
    long cur_time = System.currentTimeMillis() ;
    while ((System.currentTimeMillis() - cur_time) < 2000) {
        // display the values 
        /*
        getAngle() ;
        displayOrientationValues(lastAngles, "imu vals") ;
        telemetry.update() ;
        */
        strafeRightIMU(0.5) ;
    }
    stopAllWheelsPower();
    
    
    /*
    getAngle() ;
    displayOrientationValues(lastAngles, "imu vals") ;
    telemetry.update() ;

    rotate(-45,0.75) ;
    //sleep(1000) ;
    
    getAngle() ;
    displayOrientationValues(lastAngles, "imu vals") ;
    telemetry.update() ;
    */

    /*    
    // turn the robot  and see what changes take place in the values 
    driveFL.setPower(1.0);
    driveBL.setPower(1.0);
    driveFR.setPower(1.0);
    driveBR.setPower(1.0);
    sleep (1500) ;
    stopAllWheelsPower();
    
    // let it rest for a bit
    sleep(1000) ;

    // display the values again
    getAngle() ;
    displayOrientationValues(lastAngles, "imu end") ;

    telemetry.update() ;
    */
    
    // Experimentally, we have established that the firstAngle value is the 
    // one that changes when the robot changes direction 
    // This is crucial.. 
    // Check REV robotics documentation.. it seems a bit off 
    
    sleep(20000) ;
  }

  private void moveForwardIMU(double power) {
    
    correction = checkDirection();

    backLeft.setPower(power - correction);
    frontLeft.setPower(power - correction);
    backRight.setPower(power + correction);
    frontRight.setPower(power + correction);

  }
  
  private void moveBackwardIMU(double power) {
    
    correction = checkDirection();

    backLeft.setPower((power - correction) * -1.0);
    frontLeft.setPower((power - correction) * -1.0);
    backRight.setPower((power + correction) * -1.0);
    frontRight.setPower((power + correction) * -1.0);
    
  }
  
  private void strafeRightIMU(double power) {
    
    correction = checkDirection();

    backLeft.setPower(power - correction);
    frontLeft.setPower((power - correction) * -1.0);
    backRight.setPower((power + correction) * -1.0);
    frontRight.setPower(power + correction);

  }

  private void strafeLeftIMU(double power) {
    
    correction = checkDirection();

    backLeft.setPower((power - correction) * -1.0);
    frontLeft.setPower(power - correction);
    backRight.setPower(power + correction);
    frontRight.setPower((power + correction) * -1.0);

  }


  private void testIntakeServo(double power) {
    intakeServo.setPosition(0.0) ;
    sleep(3000) ;
    intakeServo.setPosition(1.0) ;
    sleep(3000) ;
    intakeServo.setPosition(0.0) ;
    sleep(3000) ;
  }

}