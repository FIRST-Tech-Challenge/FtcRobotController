//A basic teleOp program with basic input from controllers being able to control the motion of the robot
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.text.DecimalFormat;
@TeleOp(name = "test opmode")
public class BasicTeleOpMode extends LinearOpMode {
    DcMotor frontLeft,backLeft,frontRight,backRight;

    public void initHardware(){
        //INIT CODE GOES HERE
        frontLeft = hardwareMap.dcMotor.get("frontLeft");
        backLeft = hardwareMap.dcMotor.get("backLeft");
        frontRight = hardwareMap.dcMotor.get("frontRight");
        backRight = hardwareMap.dcMotor.get("backRight");
            /*frontLeft = hardwareMap.dcMotor.get("back_left_motor");
            backLeft = hardwareMap.dcMotor.get("front_left_motor");
            frontRight = hardwareMap.dcMotor.get("front_right_motor");
            backRight = hardwareMap.dcMotor.get("back_right_motor");*/
    }

    public void runOpMode() {
        waitForStart();
        //POST INIT CODE
        if(opModeIsActive()) {
            double wheelsPowerFactor = 0.6;
            while(opModeIsActive()) {

                //THESE ARE THE MOVEMENT FUNCTIONS
                double output_x = Math.pow(gamepad1.left_stick_x, 3);
                double output_y = Math.pow(gamepad1.left_stick_y, 3);
                double output_xRight = Math.pow(gamepad1.right_stick_x, 3);
                double drive = -output_y * wheelsPowerFactor;//vertical movement = the left stick on controller one(moving on the y-axis)
                double strafe = -(output_x * wheelsPowerFactor);//Strafing = the left stick on controller 1(moving on the x-axis)
                double rotate = output_xRight * wheelsPowerFactor;
                frontLeft.setPower(drive - (strafe - rotate));
                backLeft.setPower(drive + strafe + rotate);
                frontRight.setPower(-(drive + (strafe - rotate)));
                backRight.setPower(-(drive - (strafe + rotate)));
            }
        }
    }
}

