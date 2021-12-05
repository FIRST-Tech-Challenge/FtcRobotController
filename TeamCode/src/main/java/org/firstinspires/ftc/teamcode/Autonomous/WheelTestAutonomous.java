package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous

public class WheelTestAuto extends LinearOpMode{

    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;
    
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

            
            frontLeft.setPower(1.0) ;
            sleep(5000) ;
            frontLeft.setPower(0.0) ;
           sleep(1000) ;
           
           frontRight.setPower(1.0) ;
           sleep(5000) ;
           frontRight.setPower(0.0) ;
           sleep(1000) ;

            backLeft.setPower(1.0) ;
           sleep(5000) ;
           backLeft.setPower(0.0) ;
           sleep(1000) ;
           
           
           backRight.setPower(1.0) ;
           sleep(5000) ;
           backRight.setPower(0.0) ;
           

            /*
            frontLeft.setPower(0.5) ;
            frontRight.setPower(0.5) ;
            backLeft.setPower(0.5) ;
            backRight.setPower(0.5) ;
            
            sleep(5000) ;
            frontLeft.setPower(0.0) ;
           frontRight.setPower(0.0) ;
           backLeft.setPower(0.0) ;
           backRight.setPower(0.0) ;
           sleep(1000) ;
           
            frontLeft.setPower(-0.5) ;
            frontRight.setPower(-0.5) ;
            backLeft.setPower(-0.5) ;
            backRight.setPower(-0.5) ;
            
            sleep(5000) ;
            frontLeft.setPower(0.0) ;
           frontRight.setPower(0.0) ;
           backLeft.setPower(0.0) ;
           backRight.setPower(0.0) ;
           sleep(1000) ;
            */
            /*
            frontLeft.setPower(0.5) ;
            frontRight.setPower(-0.5) ;
            backLeft.setPower(-0.5) ;
            backRight.setPower(0.5) ;
            
            sleep(5000) ;
            frontLeft.setPower(0.0) ;
           frontRight.setPower(0.0) ;
           backLeft.setPower(0.0) ;
           backRight.setPower(0.0) ;
           sleep(1000) ;
           
            frontLeft.setPower(-0.5) ;
            frontRight.setPower(0.5) ;
            backLeft.setPower(0.5) ;
            backRight.setPower(-0.5) ;
            
            sleep(5000) ;
            frontLeft.setPower(0.0) ;
           frontRight.setPower(0.0) ;
           backLeft.setPower(0.0) ;
           backRight.setPower(0.0) ;
           sleep(1000) ;
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
    }
}
