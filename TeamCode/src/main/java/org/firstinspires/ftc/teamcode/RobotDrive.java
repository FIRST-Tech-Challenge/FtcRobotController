package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.*;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.Locale;

public class RobotDrive {
    //Proportional Processing values for distance to drive function
    private final double P_Forward = 0.05;
    private final double P_Strafe = 0.025;
    private final double P_Turn = 0.005;


    Telemetry telemetry = null;
    color teamColor = null;

    //Proportional Value used in self-correcting gyro code for encoder driving
    private final double TURN_P = 0.005;
    private final double GYRO_P = 0.01;
    private final double wheelDiameter = 3.93701;


    //Hardware
    private DcMotorEx leftFront, leftRear, rightFront, rightRear = null;
    DcMotorEx[] motors = {leftFront, leftRear, rightFront, rightRear};
    private BNO055IMU imu = null;
    public DistanceSensor dist = null;
    public ColorSensor colorSensor = null;

    //Default motor power levels for wheels
    public double motorPower = 0.5;

    //Debug the error angle in order to get this value
     private final double TURNING_BUFFER = 0;

    //
    enum direction {
        left, right;
    }

    enum color {
        red, blue;
    }

    //Assigning software objects to hardware, receives hardwareMap and telemetry objects from the op mode which calls it
    void initializeRobot(HardwareMap hardwareMap, Telemetry telem, color clr) {
        telemetry = telem;
        teamColor = clr;

        //Initialize hardware from hardware map
        leftFront = (DcMotorEx)hardwareMap.dcMotor.get("front_left_motor");
        rightFront = (DcMotorEx)hardwareMap.dcMotor.get("front_right_motor");
        leftRear = (DcMotorEx)hardwareMap.dcMotor.get("back_left_motor");
        rightRear = (DcMotorEx)hardwareMap.dcMotor.get("back_right_motor");
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        dist = hardwareMap.get(DistanceSensor.class, "distance");
        colorSensor = hardwareMap.get(ColorSensor.class, "colorSense");


        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Sensor Initialization
        if (colorSensor instanceof SwitchableLight) {
            ((SwitchableLight)colorSensor).enableLight(false);
        }

        //Motor initialization
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightRear.setDirection(DcMotor.Direction.REVERSE);

        rightFront.setPositionPIDFCoefficients(5);
        rightRear.setPositionPIDFCoefficients(5);
        leftFront.setPositionPIDFCoefficients(5);
        leftRear.setPositionPIDFCoefficients(5);


        //Initialize IMU
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.loggingEnabled = false;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        imu.initialize(parameters);

    }


    /***************************************FORWARD MOVEMENT***************************************/
    //Send this function a value of seconds to drive for and it will drive for that period
    void driveTime(long time) throws InterruptedException {
        for (DcMotor motor: motors) motor.setPower(motorPower);
        Thread.sleep(time);
        for (DcMotor motor: motors) motor.setPower(0);

        telemetry.addData("Rear Left", leftRear.getCurrentPosition());
        telemetry.addData("Front Left", leftFront.getCurrentPosition());
        telemetry.addData("Front Right", rightFront.getCurrentPosition());
        telemetry.addData("Rear Right", rightRear.getCurrentPosition());
        telemetry.update();

    }

    //Send this function a number of inches and it will drive that distance using the encoders on the motors.
    void driveEncoder(double Inches) {
        float initialHeading = getHeading();
        int encoderTicks = 0;
        if (Inches > 0) encoderTicks = (int)(480 * (float)((Inches - 1) / (wheelDiameter * Math.PI)));
        else if(Inches < 0) encoderTicks = (int)(480 * (float)((Inches + 1) / (wheelDiameter * Math.PI)));
         for (DcMotor motor: motors) {
             motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
             motor.setTargetPosition(encoderTicks);
             motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
         }

         for (DcMotor motor: motors) {
             motor.setPower(motorPower);
         }

        while (Math.abs(leftFront.getCurrentPosition() - leftFront.getTargetPosition()) > 30) {
            //wait until the motors are done running
           if (Math.abs((initialHeading - getHeading()) % 360) > 1) {
               double degreesCorrect = (initialHeading - getHeading()) % 360;
               double motorCorrect = clamp(degreesCorrect * GYRO_P, -.4, .4);
               leftFront.setPower(motorPower - motorCorrect);
               leftRear.setPower(motorPower - motorCorrect);
               rightFront.setPower(motorPower + motorCorrect);
               rightRear.setPower(motorPower + motorCorrect);
           }
                telemetry.addData("Rear Left", leftRear.getCurrentPosition());
                telemetry.addData("Front Left", leftFront.getCurrentPosition());
                telemetry.addData("Front Right", rightFront.getCurrentPosition());
                telemetry.addData("Rear Right", rightRear.getCurrentPosition());
                telemetry.update();
        }
        for (DcMotor motor : motors)
            motor.setPower(0);

        for (DcMotor motor : motors)
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    /*******************************************STRAFING*******************************************/
    //Send this function a time value as well as a direction (Ex: RobotDrive.direction.left) and it will strafe that direction for the specified amount of time
    void strafeTime(int time, direction strafeDirection) throws InterruptedException {
        if (strafeDirection == direction.left) {
            leftFront.setPower(-1 * motorPower);
            leftRear.setPower(motorPower);
            rightRear.setPower(-1 * motorPower);
            rightFront.setPower(motorPower);
        } else if (strafeDirection == direction.right) {
            leftFront.setPower(motorPower);
            leftRear.setPower(-1 * motorPower);
            rightRear.setPower(motorPower);
            rightFront.setPower(-1 * motorPower);
        }
        Thread.sleep(time);
        leftFront.setPower(0);
        leftRear.setPower(0);
        rightFront.setPower(0);
        rightRear.setPower(0);
    }
    //Send this function a distance in inches as well as a direction (Ex: RobotDrive.direction.right) and it will strafe that direction for the specified distance
    void strafeEncoder(double Inches, direction direction) {
        float initialHeading = getHeading();

        int encoderTicks = (int)(480 * (float)(Inches / (wheelDiameter * Math.PI)));
        if (direction == RobotDrive.direction.left) encoderTicks *= -1;
        for (DcMotor motor : motors) motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setTargetPosition(encoderTicks);
        rightFront.setTargetPosition(-1 * encoderTicks);
        rightRear.setTargetPosition(encoderTicks);
        leftRear.setTargetPosition(-1 *encoderTicks);
        for (DcMotor motor : motors) {
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        leftFront.setPower(motorPower - 0.1);
        rightFront.setPower(motorPower - 0.1);
        leftRear.setPower(motorPower + 0.1);
        rightRear.setPower(motorPower + 0.1);

        while (Math.abs(leftFront.getCurrentPosition() - leftFront.getTargetPosition()) > 30) {
            //wait until the motors are done running


            if (Math.abs((initialHeading - getHeading()) % 360) > 1) {
                double degreesCorrect = (initialHeading - getHeading()) % 360;
                double motorCorrect = clamp(degreesCorrect * GYRO_P, -.8, .8);
                leftFront.setPower(motorPower - motorCorrect);
                leftRear.setPower(motorPower - motorCorrect);
                rightFront.setPower(motorPower + motorCorrect);
                rightRear.setPower(motorPower + motorCorrect);
            }
            telemetry.addData("Front Left: ", leftFront.getCurrentPosition());
            telemetry.addData("Front Right: ", rightFront.getCurrentPosition());
            telemetry.addData("Back Left: ", leftRear.getCurrentPosition());
            telemetry.addData("Back Right: ", rightRear.getCurrentPosition());
            telemetry.update();
        }
        for (DcMotor motor : motors) {
            motor.setPower(0);
        }
        for (DcMotor motor : motors) {
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

    }


    /*******************************************TURNING********************************************/
    //Send this a value of degrees to turn, positive value turns right and negative value turns left, uses IMU gyroscope to precisely turn that distance
    void gyroTurn(double degrees) {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double target_angle = getHeading() - degrees;
        if (degrees < 0) {target_angle += TURNING_BUFFER;} else if (degrees > 0) {target_angle -= TURNING_BUFFER;}
        while (Math.abs((target_angle - getHeading()) % 360) > 3) {
            double error_degrees = (target_angle - getHeading()) % 360; //Compute turning error
            double motor_output = clamp(error_degrees * TURN_P, -.9, .9); // Get Correction of error
            //Send corresponding value to motors
            leftFront.setPower(-1 * motor_output);
            leftRear.setPower(-1 * motor_output);
            rightFront.setPower(motor_output);
            rightRear.setPower(motor_output);

            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            telemetry.addData("Target Angle : ", target_angle - TURNING_BUFFER);
            telemetry.addData("Current Heading : ", String.format(Locale.getDefault(), "%.1f", angles.firstAngle * -1));
            telemetry.update();
        }

        telemetry.addData("Error Degrees: ", Math.abs(target_angle - angles.firstAngle) % 360);
        telemetry.update();
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftRear.setPower(0);
        rightRear.setPower(0);
    }

    void gyroTurn(double degrees, double motorClamp) {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double target_angle = getHeading() - degrees;
        if (degrees < 0) {target_angle += TURNING_BUFFER;} else if (degrees > 0) {target_angle -= TURNING_BUFFER;}
        while (Math.abs((target_angle - getHeading()) % 360) > 3) {
            double error_degrees = (target_angle - getHeading()) % 360; //Compute turning error
            double motor_output = clamp(error_degrees * TURN_P, -motorClamp, motorClamp); // Get Correction of error
            //Send corresponding value to motors
            leftFront.setPower(-1 * motor_output);
            leftRear.setPower(-1 * motor_output);
            rightFront.setPower(motor_output);
            rightRear.setPower(motor_output);

            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            telemetry.addData("Target Angle : ", target_angle - TURNING_BUFFER);
            telemetry.addData("Current Heading : ", String.format(Locale.getDefault(), "%.1f", angles.firstAngle * -1));
            telemetry.update();
        }

        telemetry.addData("Error Degrees: ", Math.abs(target_angle - angles.firstAngle) % 360);
        telemetry.update();
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftRear.setPower(0);
        rightRear.setPower(0);
    }



    /*********************************************SERVOS*********************************************/

    /** MOST OF THESE FUNCTIONS ARE FROM SKYSTONE AND NOW DEPRECATED, BUT LEFT AROUND FOR REFERENCE IN CONTROLLING SERVOS**/

   //Activates the back servos used to grab the mat, send angle of rotation in degrees as well as the max angle of the servo. Converts to a fraction usable by the servo
   void seekMat() throws InterruptedException {
       if (colorSensor instanceof SwitchableLight){
           ((SwitchableLight)colorSensor).enableLight(true);
       }

       mixDrive(0.2, 0, 0);
       if (teamColor == color.red) {
           while (colorSensor.red() < 230);
           mixDrive(0, 0, 0);
           grabMat(90);
           Thread.sleep(500);
           mixDrive(-0.4, 0, 0);
           while (dist.getDistance(DistanceUnit.INCH) > 5);
           mixDrive(0,0,0);
           grabMat(0);
           Thread.sleep(50);
           mixDrive(0, -0.3, 0);
           //SetSideArm(70,180);
           Thread.sleep(1000);
           while (colorSensor.red() < 188);
           mixDrive(0,0,0);
           Thread.sleep(20);
           strafeEncoder(5, direction.right);
           mixDrive(0,0,0);
       }
       else {
           while (colorSensor.blue() < 205);
           mixDrive(0, 0, 0);
           grabMat(90);
           Thread.sleep(500);
           mixDrive(-0.4, 0, 0);
           while (dist.getDistance(DistanceUnit.INCH) > 4);
           mixDrive(0,0,0);
           grabMat(0);
           Thread.sleep(50);
           mixDrive(0, 0.3, 0);
           //SetSideArm(70,180);
           Thread.sleep(500);
           while (colorSensor.blue() < 190);
           mixDrive(0,0,0);
           Thread.sleep(20);
           strafeEncoder(3, direction.right);
           mixDrive(0,0,0);
       }

       if (colorSensor instanceof SwitchableLight) {
           ((SwitchableLight)colorSensor).enableLight(false);
       }
   }

    void grabMat(float desiredRotation) {
    //MatServos.setPosition(desiredRotation / 280);
   }

   void controlClaw(float desiredRotation) {
       //BlockGrips.setPosition(desiredRotation / 280);
       //TopServo.setPosition((desiredRotation + 15) / 180);
   }

    /*******************************************UTILITIES*******************************************/
    //Creating a clamp method for both floats and doubles, used to make sure motor power doesn't go above a certain power level as to saturate the motors
    public static double clamp(double val, double min, double max) {
        return Math.max(min, Math.min(max, val));
    }

    public static float clamp(float val, float min, float max) {
        return Math.max(min, Math.min(max, val));
    }

    public static int clamp(int val, int min, int max) { return Math.max(min, Math.min(max, val));
    }

    //For debugging, displays current encoder values of each wheel
    void getEncoderVals() {
        telemetry.addData("Encoders (LF, RF, LR, RR)", "%d %d %d %d",
                leftFront.getCurrentPosition(),
                rightFront.getCurrentPosition(),
                leftRear.getCurrentPosition(),
                rightRear.getCurrentPosition());
        telemetry.update();
    }

    //Returns the current heading of the robot when it is called, takes reading from the IMU gyroscope
    float getHeading() {
        return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
    }


    void resetEncoders() {
        for (DcMotor motor : motors) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        telemetry.addLine("Encoders Reset!");
        telemetry.update();
    }

    //Driving function that accepts three values for forward, strafe, and rotate, then mixes those values into 4 usable motor power outputs and sends to the motors.
    void mixDrive(double forward, double strafe, double rotate) {
        double frontLeftSpeed = clamp((forward + strafe + rotate), -motorPower, motorPower);
        double frontRightSpeed = clamp((forward - strafe - rotate), -motorPower, motorPower);
        double backLeftSpeed = clamp((forward - strafe + rotate ), -motorPower, motorPower);
        double backRightSpeed = clamp((forward + strafe - rotate), -motorPower, motorPower);

        leftFront.setPower(frontLeftSpeed);
        rightFront.setPower(frontRightSpeed);
        leftRear.setPower(backLeftSpeed);
        rightRear.setPower(backRightSpeed);
    }

    //turning distance measurements into usable motor output via Proportional control)
    void DistanceToDrive(double forward, double right, double turn){

        double ForwardOut = forward * P_Forward;
        double RightOut = right * P_Strafe;
        double TurnOut = turn * P_Turn;
        mixDrive(ForwardOut, RightOut, TurnOut);
    }

    void LiftTime(long time) throws InterruptedException {
        //armLift.setPower(liftPower);
        Thread.sleep(time);
        //armLift.setPower(0);

    }
}
