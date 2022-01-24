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

package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Control", group="Linear Opmode")
public class drive extends LinearOpMode {

    // Declare OpMode members.
    private DcMotor motorFrontLeft = hardwareMap.dcMotor.get("motorFrontLeft");//hardware
    private DcMotor motorBackLeft = hardwareMap.dcMotor.get("motorBackLeft");
    private DcMotor motorFrontRight = hardwareMap.dcMotor.get("motorFrontRight");
    private DcMotor motorBackRight = hardwareMap.dcMotor.get("motorBackRight");
    private Servo servo0 = hardwareMap.servo.get("servo0");

    //public static PIDCoefficients pidCoeffs = new PIDCoefficients(0,0,0);currently not used
    //public PIDCoefficients pidGain = new PIDCoefficients(0,0,0);
    private ElapsedTime runtime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);//gets time, used for PID
    ElapsedTime timing = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);//gets time, used for a second PID

    static double kP=0.02; //tuning factor, change these numbers around if the PID overshoots or is too slow
    static double kI=0.001;
    static double kD=0.003;

    static double skP=0.8; //tuning factor for servo
    static double skI=0.8;
    static double skD=0.5;

    static double maxSpeed= 1.5;//idk, how fast is the motor's top speed in ticks/ms. if you find out change this number to match
    static double max_i = 1;// the max amount that the integral portion of the PID influences the output by



    @Override
    public void runOpMode() {//default code from basic linearOPmode
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        motorFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);

        // Wait for the game to start (driver presses PLAY)

        //double fV=0.7;
        //double fA=1;

        waitForStart();

        if (isStopRequested()) return;

        //format: previous time, previous encoder value, previous error, integral,prev Power

        double[] fL = {0,0,0,0,0};//these arrays contain data used by the PID, the order of the data is 2 lines above
        double[] bL = {0,0,0,0,0};
        double[] fR = {0,0,0,0,0};
        double[] bR = {0,0,0,0,0};

        // double prev time, double previous_error, double integral,position
        double[] srvo = {0,0,0,0};//used for servo PID

        motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);//reseting encoders
        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);//turns off the built in PID, it'll still get measurements from the encoders
        motorBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        servo0.setPosition(0);


        runtime.reset();
        // run until the end of the match (driver presses STOP
        while (opModeIsActive()) {

            //UPDATE SPEED, finds out what percent of max speed to use from controller + direction
            double y = -gamepad1.left_stick_y*maxSpeed; // Remember, this is reversed!
            double x = gamepad1.left_stick_x*maxSpeed;
            double rx = gamepad1.right_stick_x*maxSpeed;
            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio, but only when
            // at least one is out of the range [-1, 1]

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), maxSpeed);//turns gamepad input into motor speed
            double frontLeftSpeed = (y + x + rx) / denominator;
            double backLeftSpeed = (y - x + rx) / denominator;
            double frontRightSpeed= (y - x - rx) / denominator;
            double backRightSpeed = (y + x - rx) / denominator;

            fL=PID(motorFrontLeft,frontLeftSpeed,fL[0],fL[1],fL[2],fL[3]);//runs PID on motor to get to desired motor speed
            fR=PID(motorFrontRight,frontRightSpeed,fR[0],fR[1],fR[2],fR[3]);
            bR=PID(motorBackRight,backRightSpeed,bR[0],bR[1],bR[2],bR[3]);
            bL=PID(motorBackLeft,backLeftSpeed,bL[0],bL[1],bL[2],bL[3]);

            motorFrontLeft.setPower(fL[5]);//sets motor power to PID output
            motorFrontRight.setPower(fR[5]);
            motorBackLeft.setPower(bL[5]);
            motorBackRight.setPower(bR[5]);

            int state = 0;//state 0 indicates servo is in starting position

            switch(state){ //servo
                case 0:
                    if(gamepad1.a){//if press a, servo begins turning to position 1.
                        state=1;
                    }
                    break;
                case 1:
                    if(servo0.getPosition()==1){
                        timing.reset();
                        state=2;
                    }
                    srvo=PID2(servo0,1,srvo[0],srvo[1],srvo[2]);
                    servo0.setPosition(srvo[3]);
                    break;
                case 2:
                    if(servo0.getPosition()!=1){
                        state=1;
                        break;
                    }
                    if(timing.time()>=100){
                        state=3;
                    }
                    break;
                case 3:
                    //some action over here. this part of the state machine means the servo's PID loop got it to pos:1.
                    break;

            }


            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            //telemetry.addData("Motors", "FLeft (%.2f), FRight (%.2f)", frontLeftPower, backLeftPower, frontRightPower, backRightPower);
            telemetry.update();
        }
    }


    public double[] PID(DcMotor motor, double desire_speed,double prevTime,double prevEncode, double previous_error, double integral){//the PID for motor speed

        double p; //proportional
        double d; //derivative
        double curr_encode = motor.getCurrentPosition();//gets motor position. used in calculating current speed

        double current_time = runtime.time();//gets time, used in calculating current speed
        double current_speed = (curr_encode-prevEncode)/(current_time-prevTime);//calculate the current speed
        double current_error = desire_speed-current_speed;//get error

        p = (kP * current_error)/maxSpeed;//proportional term output

        integral += kI * (current_error * (current_time - prevTime));//integral term output

        if (integral > max_i) {//sets integral term to max_i if it exceeds
            integral = max_i;
        }

        else if(integral < -max_i) {//same but for if it goes too low
            integral = -max_i;
        }

        d = (kD * (current_error - previous_error) / (current_time - prevTime))/maxSpeed;//derivative term output

        double newPower= p + integral + d;//output of PID

        double[] output = {current_time,curr_encode,current_error,integral,newPower};//sends back data to be used in next loop of PID + the motor power


        return output;

    }

    public double[] PID2(Servo servo, double desire_pos, double prevTime, double previous_error, double integral){//PID but for servo pos, pretty much the same

        double p; //proportional
        double d; //derivative



        double current_error = desire_pos-servo.getPosition();
        double current_time = runtime.time();


        p = kP * current_error;

        integral += kI * (current_error * (current_time - prevTime));

        if (integral > max_i) {
            integral = max_i;
        }
        else if(integral < -max_i) {
            integral = -max_i;
        }

        d = kD * (current_error - previous_error) / (current_time - prevTime);

        double output = p + integral + d;

        double[] result = {current_time,current_error,integral,output};
        return result;


    }

}
// end