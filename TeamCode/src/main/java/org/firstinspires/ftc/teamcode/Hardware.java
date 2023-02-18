package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


public class Hardware {

    // Motor variable names
    public DcMotor rightSlides = null;
    public DcMotor leftSlides = null;

    public CRServo leftIntake = null;
    public CRServo rightIntake = null;

    public DistanceSensor distanceSensor = null;


    // Other variable names
    public HardwareMap hwMap;

    public Hardware() {
        hwMap = null;
    }

    //called in initializeRobot method in AutonomousMethods
    public void initializeHardware(HardwareMap hwMap) {

        // Save reference to Hardware map
        this.hwMap = hwMap;

        // Define Motors

        rightSlides = hwMap.dcMotor.get("rightSlides");

        leftSlides = hwMap.dcMotor.get("leftSlides");


        // Define Servos
        leftIntake = hwMap.crservo.get("leftIntake");

        rightIntake = hwMap.crservo.get("rightIntake");

        // ******MAY CHANGE *******  Fix Forward/Reverse under testing

        //Subsystem Motors & Servos
        rightSlides.setDirection(DcMotorSimple.Direction.REVERSE);
        leftSlides.setDirection(DcMotorSimple.Direction.FORWARD);

        rightSlides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftSlides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightSlides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftSlides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //SENSORS
        distanceSensor = hwMap.get(DistanceSensor.class, "distance");

        rightSlides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftSlides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    public void sleep(double time){
        ElapsedTime timer = new ElapsedTime();
        while(timer.seconds()<(time/1000)){

        }

    }
}