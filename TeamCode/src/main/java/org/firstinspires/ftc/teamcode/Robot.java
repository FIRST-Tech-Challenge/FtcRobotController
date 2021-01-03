package org.firstinspires.ftc.teamcode;



import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;


import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

public class Robot {

    public DcMotor Right_Front_Wheel;
    public DcMotor Left_Front_Wheel;
    public DcMotor Right_Rear_Wheel;
    public DcMotor Left_Rear_Wheel;

    //IMU Sensor
    private BNO055IMU imu;

    private Telemetry telemetry;

    private HardwareMap hardwareMap;

    private boolean initializeCamera = false;

    public int State = 0;

    public void Init(HardwareMap hardwareMap, Telemetry telemetry, boolean initializeCamera) {
        this.telemetry = telemetry;
        this.hardwareMap = hardwareMap;
        this.initializeCamera = initializeCamera;

        //Initialize IMU hardware map value.
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        // Create an IMU parameters object.
        BNO055IMU.Parameters IMU_Parameters = new BNO055IMU.Parameters();
        // Set the IMU sensor mode to IMU. This mode uses
        // the IMU gyroscope and accelerometer to
        // calculate the relative orientation of hub and
        // therefore the robot.
        IMU_Parameters.mode = BNO055IMU.SensorMode.IMU;
        // Initialize the IMU using parameters object.
        imu.initialize(IMU_Parameters);

        Right_Front_Wheel = hardwareMap.dcMotor.get("Right_Front_Wheel");
        Left_Front_Wheel = hardwareMap.dcMotor.get("Left_Front_Wheel");
        Right_Rear_Wheel = hardwareMap.dcMotor.get("Right_Rear_Wheel");
        Left_Rear_Wheel = hardwareMap.dcMotor.get("Left_Rear_Wheel");
/*
        Right_Front_Wheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Right_Rear_Wheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Left_Front_Wheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Left_Rear_Wheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        Right_Front_Wheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Right_Rear_Wheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Left_Front_Wheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Left_Rear_Wheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        Right_Front_Wheel.setDirection(DcMotorSimple.Direction.REVERSE);
        Right_Rear_Wheel.setDirection(DcMotorSimple.Direction.REVERSE);
 */
    }

    public void Stop(){
        Left_Front_Wheel.setPower(0);
        Right_Front_Wheel.setPower(0);
        Left_Rear_Wheel.setPower(0);
        Right_Rear_Wheel.setPower(0);
    }

    public void Forward(float power) {
        Left_Front_Wheel.setPower(power);
        Right_Front_Wheel.setPower(power);
        Right_Rear_Wheel.setPower(power);
        Left_Rear_Wheel.setPower(power);
    }

    public void Reverse(float power) {
        Left_Front_Wheel.setPower(power * -1);
        Right_Front_Wheel.setPower(power * -1);
        Right_Rear_Wheel.setPower(power * -1);
        Left_Rear_Wheel.setPower(power * -1);
    }

    public void Right(float power) {
        Left_Front_Wheel.setPower(power);
        Right_Front_Wheel.setPower(power * -1);
        Right_Rear_Wheel.setPower(power * -1);
        Left_Rear_Wheel.setPower(power);
    }

    public void Left(float power) {
        Left_Front_Wheel.setPower(power * -1);
        Right_Front_Wheel.setPower(power);
        Right_Rear_Wheel.setPower(power);
        Left_Rear_Wheel.setPower(power * -1);
    }

    public void Diagonal_Right_Up(float power) {
        Left_Front_Wheel.setPower(power);
        Right_Front_Wheel.setPower(0);
        Right_Rear_Wheel.setPower(power);
        Left_Rear_Wheel.setPower(0);

    }

    public void Diagonal_Left_Down(float power) {
        Left_Front_Wheel.setPower(power * -1);
        Right_Front_Wheel.setPower(0);
        Right_Rear_Wheel.setPower(power * -1);
        Left_Rear_Wheel.setPower(0);

    }

    public void Diagonal_Right_Down(float power) {
        Left_Front_Wheel.setPower(0);
        Right_Front_Wheel.setPower(power * -1);
        Right_Rear_Wheel.setPower(0);
        Left_Rear_Wheel.setPower(power);

    }

    public void Diagonal_Left_Up(float power) {
        Left_Front_Wheel.setPower(0);
        Right_Front_Wheel.setPower(power);
        Right_Rear_Wheel.setPower(0);
        Left_Rear_Wheel.setPower(power);
    }

    public void Slide_Right(float power) {
        Left_Front_Wheel.setPower(power);
        Right_Front_Wheel.setPower(power * -1);
        Right_Rear_Wheel.setPower(power);
        Left_Rear_Wheel.setPower(power * -1);

    }

    public void Slide_Left(float power) {
        Left_Front_Wheel.setPower(power * -1);
        Right_Front_Wheel.setPower(power);
        Right_Rear_Wheel.setPower(power * -1);
        Left_Rear_Wheel.setPower(power);
    }
}
