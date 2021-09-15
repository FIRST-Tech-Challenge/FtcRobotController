package org.firstinspires.ftc.team6220_2020;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.team6220_2020.ResourceClasses.PIDFilter;

import java.util.ArrayList;
import java.util.List;

// todo - add is op mode active breakers
public abstract class MasterAutonomous extends MasterOpMode {

    public void turnDegrees(double targetAngle) {
        motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        double startAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        double angleLeft;
        double angleTraveled;

        boolean angleReached = false;

        PIDFilter translationPID;
        translationPID = new PIDFilter(Constants.ROTATION_P, Constants.ROTATION_I, Constants.ROTATION_D);

        while (!angleReached && opModeIsActive()) {
            // This gets the angle change
            double currentAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
            angleTraveled = currentAngle - startAngle;

            // This adds a value to the PID loop so it can update
            angleLeft = targetAngle - angleTraveled;
            translationPID.roll(angleLeft);

            // We drive the mecanum wheels with the PID value
            driveMecanum(0.0, 0.0, Math.max((translationPID.getFilteredValue() / 5), Constants.MINIMUM_TURNING_POWER));

            if (Math.abs(targetAngle - angleTraveled) < 1) {
                driveMecanum(0.0, 0.0, 0.0);
                angleReached = true;
            }
        }
    }

    public void turnToAngle(double targetAngle){

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "AdafruitIMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        pauseMillis(1000);

        while(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle  <= targetAngle - 1){

            //todo add proprotional turning.
            driveMecanum(0,0,-0.2);
            telemetry.addData("IMU", imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
            telemetry.update();

        }

        driveMecanum(0,0,0);

        //double startAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
//
        //if(targetAngle < startAngle){
        //    while (imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle - targetAngle <= 1 && opModeIsActive()){
        //        driveMecanum(0,0,-0.2);
        //    }
        //} else{
        //    while (imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle - targetAngle <= 1 && opModeIsActive()){
        //        driveMecanum(0,0,0.2);
        //    }
        //}


    }

    public int findRingStackSize(double millis){
        List<Integer> reportedHeights = new ArrayList<>();

        for(int i = 0; i < millis / 100; i++){

            reportedHeights.add(new Integer(RingDetectionPipeline.ringStackHeight));

            sleep(100);

        }

        int total = 0;
        for(int i = 0; i < reportedHeights.size() - 1; i++){
            total += reportedHeights.get(1).intValue();;
        }

        Double average = ((double) total / (double)  reportedHeights.size()) + 0.5;

        return average.intValue();

    }

}