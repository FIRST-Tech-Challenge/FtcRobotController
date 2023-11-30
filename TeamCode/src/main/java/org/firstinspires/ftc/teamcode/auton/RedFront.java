package org.firstinspires.ftc.teamcode.auton;

// import org.firstinspires.ftc.teamcode.auton.Move;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import java.util.Locale;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.ServoImpl;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import java.util.Map;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.CRServoImpl;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

@Autonomous(name = "redFrong", group = "Robot")
public class RedFront extends LinearOpMode {


    private DcMotor.ZeroPowerBehavior brake = DcMotor.ZeroPowerBehavior.BRAKE;
    private DcMotor.ZeroPowerBehavior floatt =DcMotor.ZeroPowerBehavior.FLOAT;
    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private Servo leftClawRotator;
    private Servo rightClawRotator;
    private Servo airplaneLauncher;

    private Servo leftClawServo;
    private Servo rightClawServo;

    private DcMotor craneArm;
    // private Servo goodServo;
    //private Servo badServo;
    
    private int mid = 80;
    private int turn = 65;

    
        // private Servo clawRotator;

    
    private ColorSensor sensorColor;
    
    int errorBound = 60;


    @Override
    public void runOpMode() {
        frontLeft = hardwareMap.get(DcMotor.class, "fruntLeft");
        frontRight = hardwareMap.get(DcMotor.class, "fruntRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "jarmy");
        craneArm = hardwareMap.get(DcMotor.class, "craneArm");
        
        rightClawRotator = hardwareMap.get(Servo.class, "rightClawRotator");
        leftClawRotator = hardwareMap.get(Servo.class, "leftClawRotator");
        
        rightClawServo = hardwareMap.get(Servo.class, "rightClawServo");
        leftClawServo = hardwareMap.get(Servo.class, "leftClawServo");

        // goodServo = hardwareMap.get(Servo.class, "goodServo");
        // badServo = hardwareMap.get(Servo.class, "badServo");
        
        // clawRotator = hardwareMap.get(Servo.class, "clawRotator");
        
        
        
        // rotate(0.5,turn);
        //clawRotator.setPosition(0);
        //starting posistions
        
        // pickupPixel();
        // sleep(1000);
        // openClaw();

        craneArm = hardwareMap.get(DcMotor.class, "craneArm");
        craneArm.setDirection(DcMotor.Direction.REVERSE);
        craneArm.setTargetPosition(0);
        craneArm.setPower(0.2);
        craneArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        craneArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setTargetPosition(0);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setTargetPosition(0);
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setTargetPosition(0);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setTargetPosition(0);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        
        // clawRotator.setPosition(0.0);
        
        airplaneLauncher = hardwareMap.get(Servo.class, "airplaneLauncher");


        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        
        closeClaw();
        leftClawRotator.setPosition(1);
        rightClawRotator.setPosition(0);
        airplaneLauncher.setPosition(0);

        // closeClaw();
        waitForStart();
        if(opModeIsActive()){
        
        
        //         craneArm.setPower(0.1);
        
        
        
        
        // craneArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // craneArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        //closeClaw();-1
        
        foward(-1300);
        sleep(2000);
        resetEncoders();
        
        foward(200);
        sleep(1000);
        resetEncoders();

        rotate(1100);
        sleep(2000);
        resetEncoders();
        
        foward(-1250);
        sleep(2000);
        resetEncoders();
        
        neutral();
        sleep(1000);
        
        placePixelLow();
        sleep(2000);
        foward(-300);
        sleep(2000);
        resetEncoders();
        
        openClaw();
        
        sleep(1000);
        
        pickupPixel();
        
        sleep(2000);
        // middlePush();
        // sleep(100);

        
    }
}
    public void resetEncoders(){
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


    }
    
    public void foward(int distance){
        moveVertically(frontLeft, distance, 0.5);
        moveVertically(frontRight, distance, 0.5);
        moveVertically(backRight, distance, 0.5);
        moveVertically(backLeft, distance, 0.5);
    }
    
    public void side(int distance){
        moveVertically(frontLeft, distance, 0.5);
        moveVertically(frontRight, -distance, 0.5);
        moveVertically(backRight, distance, 0.5);
        moveVertically(backLeft, -distance, 0.5);
    }
    
    public void rotate(int distance){
        moveVertically(frontLeft, distance, 0.5);
        moveVertically(frontRight, -distance, 0.5);
        moveVertically(backRight, -distance, 0.5);
        moveVertically(backLeft, distance, 0.5);
    }
    
    public void middlePush(){
        
        
    }
    /*

    public void placePixelLow(){
        craneArm.setTargetPosition(720);
        clawRotator.setPosition(0);
    }
    public void neutral(){
        craneArm.setTargetPosition(100);
        clawRotator.setPosition(0.15);
    }
    public void pickupPixel(){
        craneArm.setTargetPosition(5);
        clawRotator.setPosition(0.75);
    }
    public void openClaw(){
        goodServo.setPosition(0.35);
        badServo.setPosition(0.6);
    }
    
    public void closeClaw() {
        goodServo.setPosition(0.6);
        badServo.setPosition(0.3);
    }
    */
    
    public void moveVertically(DcMotor mot, int position, double power){
        mot.setPower(0);
        mot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mot.setTargetPosition(0);
        mot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        mot.setPower(0);
        
        mot.setTargetPosition(position);
        mot.setPower(power);
    }
    
    public void placePixelLow(){
        craneArm.setTargetPosition(1440);
        leftClawRotator.setPosition(1);
        rightClawRotator.setPosition(0);
    }
    public void neutral(){
        craneArm.setTargetPosition(200);
        leftClawRotator.setPosition(0.3);
        rightClawRotator.setPosition(0.7);
    }
    public void pickupPixel(){
        craneArm.setTargetPosition(0);
        leftClawRotator.setPosition(0.38);
        rightClawRotator.setPosition(0.62);
    }
    public void openClaw(){
        leftClawServo.setPosition(1);
        rightClawServo.setPosition(0);
    }
    
    public void closeClaw() {
        leftClawServo.setPosition(0);
        rightClawServo.setPosition(1);
    }
    
    // public void waitforMotor(DcMotor Drawer1) {
    //     while(!((Drawer1.getCurrentPosition() > Drawer1.getTargetPosition() - errorBound) && (Drawer1.getCurrentPosition() < Drawer1.getTargetPosition() + errorBound()));
    // }
    
}


