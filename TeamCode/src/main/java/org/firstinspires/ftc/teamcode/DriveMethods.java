package org.firstinspires.ftc.teamcode;



import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import static org.firstinspires.ftc.teamcode.Variables.*;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


public class DriveMethods extends LinearOpMode{

    @Override
    public void runOpMode() {}

    /*
    public void driveForDistance(double distanceMeters, boolean doStrafe, double power) { // distance: 2, strafe: false, power: 0.5
        motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        double distanceTraveled = 0;
        int targetPos = (int) (distanceMeters * clicksPerRotation * rotationsPerMeter);
        motorFL.setTargetPosition((targetPos));
        motorBL.setTargetPosition((targetPos));
        motorFR.setTargetPosition((targetPos));
        motorBR.setTargetPosition((targetPos));



        motorFL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        if (doStrafe) {
            motorFL.setPower(power);
            motorBL.setPower(-power);
            motorFR.setPower(-power);
            motorBR.setPower(power);
        } else {
            motorFL.setPower(power);
            motorBL.setPower(power);
            motorFR.setPower(power);
            motorBR.setPower(power);
        }
        //targetPos = motorFL.getTargetPosition();
        int currentPos = motorFL.getCurrentPosition();
        int FLPosition;
        int BLPosition;
        int FRPosition;
        int BRPosition;
        int avgPosition = 0;
        boolean hasNotReachedTarget = true;
        while (targetPos >= avgPosition) {
            FLPosition = Math.abs(motorFL.getCurrentPosition());
            BLPosition = Mat
            h.abs(motorBL.getCurrentPosition());
            FRPosition = Math.abs(motorFR.getCurrentPosition());
            BRPosition = Math.abs(motorBR.getCurrentPosition());
            avgPosition = (int)(FLPosition + BLPosition + FRPosition + BRPosition)/4;
            telemetry.addLine("Current Position: " + avgPosition);                      
            telemetry.addLine("targetPos " + targetPos);
            telemetry.update();
        }
        motorFL.setPower(0);
        motorBL.setPower(0);
        motorFR.setPower(0);
        motorBR.setPower(0);
    }
    */
    public void stopMotors() {
        motorFL.setPower(0);
        motorBL.setPower(0);
        motorFR.setPower(0);
        motorBR.setPower(0);
    }
    /*
    public void driveForDistance(double distanceMeters, Direction movementDirection, double power) { // distance: 2, strafe: false, power: 0.5
        motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        double distanceTraveled = 0;
        int targetPos = (int) ((distanceMeters * clicksPerRotation * rotationsPerMeter)/1.15);
        motorFL.setTargetPosition((targetPos));
        motorBL.setTargetPosition((targetPos));
        motorFR.setTargetPosition((targetPos));
        motorBR.setTargetPosition((targetPos));



        motorFL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        power = Math.abs(power);
        switch(movementDirection) {
            case FORWARD:
                motorFL.setPower(power);
                motorBL.setPower(power);
                motorFR.setPower(power);
                motorBR.setPower(power);
                break;
            case BACKWARD:
                motorFL.setPower(-power);
                motorBL.setPower(-power);
                motorFR.setPower(-power);
                motorBR.setPower(-power);
                break;
            case RIGHT:
                motorFL.setPower(power);
                motorBL.setPower(-power);
                motorFR.setPower(-power);
                motorBR.setPower(power);
                break;
            case LEFT:
                motorFL.setPower(-power);
                motorBL.setPower(power);
                motorFR.setPower(power);
                motorBR.setPower(-power);
                break;
            case ROTATE_LEFT:
                motorFL.setPower(-power);
                motorBL.setPower(-power);
                motorFR.setPower(power);
                motorBR.setPower(power);
                break;
            case ROTATE_RIGHT:
                motorFL.setPower(power);
                motorBL.setPower(power);
                motorFR.setPower(-power);
                motorBR.setPower(-power);
                break;
                
        }

        //if (doStrafe) {
        //    motorFL.setPower(power);
        //    motorBL.setPower(-power);
        //    motorFR.setPower(-power);
        //    motorBR.setPower(power);
        //} else {
        //    motorFL.setPower(power);
        //    motorBL.setPower(power);
        //    motorFR.setPower(power);
        //    motorBR.setPower(power);
        //}
        //targetPos = motorFL.getTargetPosition();

        int currentPos = motorFL.getCurrentPosition();
        int FLPosition;
        int BLPosition;
        int FRPosition;
        int BRPosition;
        int avgPosition = 0;
        boolean hasNotReachedTarget = true;
        while(targetPos >= avgPosition) {
            FLPosition = Math.abs(motorFL.getCurrentPosition());
            BLPosition = Math.abs(motorBL.getCurrentPosition());
            FRPosition = Math.abs(motorFR.getCurrentPosition());
            BRPosition = Math.abs(motorBR.getCurrentPosition());
            avgPosition = (int)(FLPosition + BLPosition + FRPosition + BRPosition)/4;
            telemetry.addLine("Current Position: " + avgPosition);                      
            telemetry.addLine("targetPos " + targetPos);
            telemetry.update();
        }
        motorFL.setPower(0);
        motorBL.setPower(0);
        motorFR.setPower(0);
        motorBR.setPower(0);
    }
    */
    public void CalibrateIMU() {

    BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
    parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
    parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
    parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
    parameters.loggingEnabled = true;
    parameters.loggingTag = "IMU";
    parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

    // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
    // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
    // and named "imu".
    imu = hardwareMap.get(BNO055IMU.class, "imu");
    imu.initialize(parameters);

    telemetry.addLine("imu should be calibrated!");
    telemetry.update();
    isImuCalibrated = true;
    sleep(1000);


    }

    public double getCurrentZ() {
        if(!isImuCalibrated){
            CalibrateIMU();
        }

        Orientation currentAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.DEGREES);
        double currentZ = currentAngle.firstAngle;
        return currentZ;
    }
    public double getCumulativeZ(){
        if(!isImuCalibrated){
            CalibrateIMU();

        }
        double currentHeading = getCurrentZ();
        double deltaHeading = currentHeading - previousHeading;
        if(deltaHeading <= -180) {
            deltaHeading += 360;
        } else if(deltaHeading >= 180) {
            deltaHeading -=360;
        }

        intergratedHeading += deltaHeading;
        previousHeading = currentHeading;

        return intergratedHeading;

    }
    //double targetZ = getCurrentZ();
    public void recenterRobotZRotation(double targetRotationZ) {
        double FLPower = motorFL.getPower();
        double BLPower = motorBL.getPower();
        double FRPower = motorFR.getPower();
        double BRPower = motorBR.getPower();
        targetZ = getCurrentZ();
        while(Math.floor(getCurrentZ()) != targetZ) {
            double currentZ = getCurrentZ();
            double rotateError = targetZ - currentZ;
            //motorFL.setPower();
            motorFL.setPower(FLPower - (rotateError / 100));
            motorBL.setPower(BLPower - (rotateError / 100));
            motorFR.setPower(FRPower + (rotateError / 100));
            motorBR.setPower(BRPower + (rotateError / 100));
        }
    }
    public void driveForDistance(double distanceMeters, Direction movementDirection, double power, double targetRotation) { // distance: 2, strafe: false, power: 0.5
        //targetZ = getCurrentZ();
        motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        double distanceTraveled = 0;
        int targetPos = (int) ((distanceMeters * clicksPerRotation * rotationsPerMeter) / 1.15);

        motorFL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        power = Math.abs(power);
        switch (movementDirection) {
            case FORWARD:
                motorFL.setPower(power);
                motorBL.setPower(power);
                motorFR.setPower(power);
                motorBR.setPower(power);
                //targetZ = 0;
                break;
            case BACKWARD:
                motorFL.setPower(-power);
                motorBL.setPower(-power);
                motorFR.setPower(-power);
                motorBR.setPower(-power);
                //targetZ = 0;
                break;
            case RIGHT:
                motorFL.setPower(power);
                motorBL.setPower(-power);
                motorFR.setPower(-power);
                motorBR.setPower(power);
                //targetZ = getCurrentZ();
                break;
            case LEFT:
                motorFL.setPower(-power);
                motorBL.setPower(power);
                motorFR.setPower(power);
                motorBR.setPower(-power);
                //targetZ = getCurrentZ();
                break;
            case ROTATE_LEFT:
                 motorFL.setPower(-power);
                 motorBL.setPower(-power);
                 motorFR.setPower(power);
                 motorBR.setPower(power);
                // targetZ = getCurrentZ();

                break;
            case ROTATE_RIGHT:
                 motorFL.setPower(power);
                 motorBL.setPower(power);
                 motorFR.setPower(-power);
                 motorBR.setPower(-power);
                // targetZ = getCurrentZ();

                break;
            default:
                motorFL.setPower(power);
                motorBL.setPower(power);
                motorFR.setPower(power);
                motorBR.setPower(power);
                break;
        }
        /*
        if(rotateToTargetRotation) {
            targetZ = targetRotation;
        }
        */
        int currentPos = 0;
        int FLPosition;
        int BLPosition;
        int FRPosition;
        int BRPosition;
        int avgPosition = 0;
        double FLPower = motorFL.getPower();
        double BLPower = motorBL.getPower();
        double FRPower = motorFR.getPower();
        double BRPower = motorBR.getPower();

        while (targetPos >= avgPosition) {
            FLPosition = Math.abs(motorFL.getCurrentPosition());
            BLPosition = Math.abs(motorBL.getCurrentPosition());
            FRPosition = Math.abs(motorFR.getCurrentPosition());
            BRPosition = Math.abs(motorBR.getCurrentPosition());

            //double currentZ = getCurrentZ();
            //double rotateError = targetZ - currentZ;

            avgPosition = (int) (FLPosition + BLPosition + FRPosition + BRPosition) / 4;
//            motorFL.setPower(FLPower - (rotateError / 150));
//            motorBL.setPower(BLPower - (rotateError / 150));
//            motorFR.setPower(FRPower + (rotateError / 150));
//            motorBR.setPower(BRPower + (rotateError / 150));

            telemetry.addLine("MotorFL Power " + motorFL.getPower());
            telemetry.addLine("MotorBL Power " + motorBL.getPower());
            telemetry.addLine("MotorFR Power " + motorFR.getPower());
            telemetry.addLine("MotorBR Power " + motorBR.getPower());

            telemetry.addLine("Current Position: " + avgPosition);
            telemetry.addLine("targetPos " + targetPos);

//            telemetry.addLine("Cumulative Z " + getCumulativeZ());
//            telemetry.addLine("Current Z " + getCurrentZ());
//            telemetry.addLine("Error " + rotateError);
            telemetry.update();
        }

        motorFL.setPower(0);
        motorBL.setPower(0);
        motorFR.setPower(0);
        motorBR.setPower(0);
    }

    public void initMotorsSecondBot() {
        motorFL  = hardwareMap.get(DcMotor.class, "motorFL");
        motorBL = hardwareMap.get(DcMotor.class, "motorBL");
        motorFR  = hardwareMap.get(DcMotor.class, "motorFR");
        motorBR = hardwareMap.get(DcMotor.class, "motorBR");
        
        motorFL.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBL.setDirection(DcMotorSimple.Direction.FORWARD);
        motorFR.setDirection(DcMotorSimple.Direction.FORWARD);
        motorBR.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void initMotorsBlue() {
        motorFL  = hardwareMap.get(DcMotor.class, "motorFL");
        motorBL = hardwareMap.get(DcMotor.class, "motorBL");
        motorFR  = hardwareMap.get(DcMotor.class, "motorFR");
        motorBR = hardwareMap.get(DcMotor.class, "motorBR");

        motorFL.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBL.setDirection(DcMotorSimple.Direction.REVERSE);
        motorFR.setDirection(DcMotorSimple.Direction.FORWARD);
        motorBR.setDirection(DcMotorSimple.Direction.FORWARD);
    }
}
