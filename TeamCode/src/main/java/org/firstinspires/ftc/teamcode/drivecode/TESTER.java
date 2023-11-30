package org.firstinspires.ftc.teamcode.drivecode;

import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad.*;

import com.qualcomm.robotcore.hardware.ServoImpl;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import java.util.Map;
import java.lang.Math;

import org.firstinspires.ftc.robotcore.external.navigation.Position;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.CRServoImpl;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.util.ElapsedTime.Resolution;
import com.qualcomm.robotcore.hardware.TouchSensor;

@TeleOp(name="TESTER", group="Linear Opmode")

public class TESTER extends LinearOpMode{
    private ZeroPowerBehavior brake = ZeroPowerBehavior.BRAKE;
    private ZeroPowerBehavior floatt = ZeroPowerBehavior.FLOAT;
    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private Servo rightClawRotator, leftClawRotator, rightClawServo, leftClawServo;
    
    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        
        frontLeft = hardwareMap.get(DcMotor.class, "fruntLeft");
        frontRight = hardwareMap.get(DcMotor.class, "fruntRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "jarmy");
        backRight = hardwareMap.get(DcMotor.class, "jarmy");

        
        rightClawRotator = hardwareMap.get(Servo.class, "rightClawRotator");
        leftClawRotator = hardwareMap.get(Servo.class, "leftClawRotator");
        
        rightClawServo = hardwareMap.get(Servo.class, "rightClawServo");
        leftClawServo = hardwareMap.get(Servo.class, "leftClawServo");
        

        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setTargetPosition(0);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setZeroPowerBehavior(ZeroPowerBehavior.FLOAT);
        
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setTargetPosition(0);
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontLeft.setZeroPowerBehavior(ZeroPowerBehavior.FLOAT);
        
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setTargetPosition(0);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setZeroPowerBehavior(ZeroPowerBehavior.FLOAT);
        
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setTargetPosition(0);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setZeroPowerBehavior(ZeroPowerBehavior.FLOAT);
        
        // craneArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // craneArm.setTargetPosition(0);
        // craneArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // craneArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        
        telemetry.addData("right position:", rightClawRotator.getPosition());
        telemetry.addData("left position:", leftClawRotator.getPosition());
        // telemetry.addData("frontLeft position:", frontLeft.getCurrentPosition());
        // telemetry.addData("backRight position:", backRight.getCurrentPosition());
        // telemetry.addData("backLeft position:", backLeft.getCurrentPosition());
        telemetry.update();
        
        waitForStart();
        
        // telemetry.addData("frontRight position:", frontRight.getCurrentPosition());
        // telemetry.addData("frontLeft position:", frontLeft.getCurrentPosition());
        // telemetry.addData("backRight position:", backRight.getCurrentPosition());
        // telemetry.addData("backLeft position:", backLeft.getCurrentPosition
        
        waitForStart();
        
        while(opModeIsActive()){
            telemetry.addData("leftx", gamepad1.left_stick_x);
            telemetry.addData("lefty", gamepad1.left_stick_y);
            telemetry.addData("leftx2", gamepad2.left_stick_x);
            telemetry.addData("lefty2", gamepad2.left_stick_y);
            telemetry.update();
        }
                
        

        




    }

}