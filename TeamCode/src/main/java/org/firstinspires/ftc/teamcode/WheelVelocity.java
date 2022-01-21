package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.robotcore.external.Telemetry ;

@Autonomous

public class WheelVelocity extends LinearOpMode{

    private DcMotorEx frontLeft;
    private DcMotorEx frontRight;
    private DcMotorEx backLeft;
    private DcMotorEx backRight;
    
    @Override
    public void runOpMode() {
        
        double curvel = 0.0, sumvel = 0.0 ; 
        int counter ;

        initializeRobot();
        
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        if (opModeIsActive()) {
            
            telemetry.addData("Status", "Running");
            telemetry.update();
            
            sleep(5000) ; // a little time to pick up the robot
            
            // for front right             
            frontLeft.setPower(1.0) ; // set full power 
            curvel = 0.0 ;
            sumvel = 0.0 ;
            for (counter=0;counter<5;counter=counter+1) {
                sleep(1000) ;
                curvel = frontLeft.getVelocity() ; 
                sumvel = sumvel + curvel ; 
                telemetry.addData("FrontLeft :", "Velocity " + (counter+1) + ": " + curvel) ;
            }
            frontLeft.setPower(0.0) ;
            telemetry.addData("FrontLeft :", "Average " +  sumvel/5) ;
            sleep(2000);
            
            frontRight.setPower(1.0) ; // set full power 
            curvel = 0.0; sumvel = 0.0; ;
            for (counter=0;counter<5;counter=counter+1) {
                sleep(1000) ;
                curvel = frontRight.getVelocity() ; 
                sumvel = sumvel + curvel ; 
                telemetry.addData("FrontRight :", "Velocity " + (counter+1) + ": " + curvel) ;
            }
            
            frontRight.setPower(0.0) ;
            telemetry.addData("FrontRight :", "Average " +  sumvel/5) ;
            sleep(2000);
            
            backLeft.setPower(1.0) ; // set full power 
            curvel = 0.0; sumvel = 0.0; ;
            for (counter=0;counter<5;counter=counter+1) {
                sleep(1000) ;
                curvel = backLeft.getVelocity() ; 
                sumvel = sumvel + curvel ; 
                telemetry.addData("backLeft :", "Velocity " + (counter+1) + ": " + curvel) ;
            }
            backLeft.setPower(0.0) ;
            telemetry.addData("backLeft :", "Average " +  sumvel/5) ;
            sleep(2000);
            
            backRight.setPower(1.0) ; // set full power 
            curvel = 0.0; sumvel = 0.0; ;
            for (counter=0;counter<5;counter=counter+1) {
                sleep(1000) ;
                curvel = backRight.getVelocity() ; 
                sumvel = sumvel + curvel ; 
                telemetry.addData("backRight :", "Velocity " + (counter+1) + ": " + curvel) ;
            }
            
            backRight.setPower(0.0) ;
            telemetry.addData("backRight :", "Average " +  sumvel/5) ;
            telemetry.update() ;
            sleep(30000);
        }
    }
            
    
    private void initializeRobot() {
        
        frontLeft = hardwareMap.get(DcMotorEx.class,"FrontLeft");
        frontRight = hardwareMap.get(DcMotorEx.class,"FrontRight");
        backLeft = hardwareMap.get(DcMotorEx.class,"BackLeft");
        backRight = hardwareMap.get(DcMotorEx.class,"BackRight");
        
        frontLeft.setDirection(DcMotorEx.Direction.FORWARD) ;
        frontRight.setDirection(DcMotorEx.Direction.REVERSE) ;
        backLeft.setDirection(DcMotorEx.Direction.FORWARD) ;
        backRight.setDirection(DcMotorEx.Direction.REVERSE) ;
        
        frontLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER) ;
        frontRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER) ;
        backLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER) ;
        backRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER) ;
 
        frontLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER) ;
        frontRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER) ;
        backLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER) ;
        backRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER) ;
       
    }
}
