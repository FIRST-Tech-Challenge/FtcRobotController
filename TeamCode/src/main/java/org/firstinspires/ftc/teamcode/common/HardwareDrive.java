/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.common;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;

import org.firstinspires.ftc.teamcode.common.ConstantsPKG.Constants;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left  drive motor:        "left_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 * Motor channel:  Manipulator drive motor:  "left_arm"
 * Servo channel:  Servo to open left claw:  "left_hand"
 * Servo channel:  Servo to open right claw: "right_hand"
 */
public class HardwareDrive
{

    public DcMotorEx  topL;
    public DcMotorEx  botR;
    public DcMotorEx  topR;
    public DcMotorEx  botL;
    public DcMotorEx  armBase;
    public DcMotorEx armTop;

//    public Servo armServo;
//    public Servo claw;

//    public DcMotorEx[] dtMotors;

    /*
    Top Left  0                              Top Right 2


    Bottom Left 1                           Bottom Right 3

     */

    //imu
    public BNO055IMU imu; //delete if we don't use it

    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();
    Constants constants = new Constants();

    /* Constructor */
    public HardwareDrive(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors

        topL = hwMap.get(DcMotorEx.class, "top_left");
        botL = hwMap.get(DcMotorEx.class, "bottom_left");
        topR = hwMap.get(DcMotorEx.class, "top_right");
        botR = hwMap.get(DcMotorEx.class, "bottom_right");
        armBase = hwMap.get(DcMotorEx.class, "arm_base");
        armTop = hwMap.get(DcMotorEx.class, "arm_top");

//        armServo = hwMap.get(Servo.class, "arm_servo");
//        claw = hwMap.get(Servo.class, "claw");


        //IMU initiation
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hwMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);


        //Set Motor Directions
        botL.setDirection(DcMotorEx.Direction.FORWARD); //no clue if these 2 are supposed to be FORWARD
        topL.setDirection(DcMotorEx.Direction.FORWARD);
        botR.setDirection(DcMotorEx.Direction.FORWARD);
        topR.setDirection(DcMotorEx.Direction.FORWARD);
//        armBase.setDirection(DcMotorEx.Direction.FORWARD);

       // dtMotors[2].setDirection(DcMotorSimple.Direction.FORWARD);
        //dtMotors[3].setDirection(DcMotorSimple.Direction.FORWARD);

        // Set all motors to zero power
        setMotorPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        setRunMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        setRunMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

    }

    public void setMotorPower(double power){
        if (power == 0.0){
            // Grady Conwell Was Here
            botL.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            topL.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            botR.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            topR.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
//            armBase.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
//            armTop.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        } else {
            botL.setPower(0);
            topL.setPower(0);
            botR.setPower(0);
            topR.setPower(0);
//            armBase.setPower(0);
//            armTop.setPower(0);
        }
    }

    public void setRunMode(DcMotorEx.RunMode runState){
        botL.setMode(runState);
        topL.setMode(runState);
        topR.setMode(runState);
        botR.setMode(runState);
        //make sure to not add arm here
    }


    public boolean wheelsAreBusy(){
        return (topL.isBusy() && botL.isBusy() && topR.isBusy() && botR.isBusy());
    }


}

