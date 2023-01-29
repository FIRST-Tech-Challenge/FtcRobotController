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

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

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
 * Motor channel:  Front Left  drive motor:        "motorFrontLeft"
 * Motor channel:  Front Right drive motor:        "motorFrontRight"
 * Motor channel:  Back Left drive motor:        "motorBackLeft"
 * Motor channel:  Back Right drive motor:        "motorBackRight"
 * Motor channel:  Manipulator drive motor:  "Arm"
 * Servo channel:  Servo to open left claw:  "leftHand"
 * Servo channel:  Servo to open right claw: "rightHand"
 * Motor channel:  Carousel Spin: "motorCarousel"
 */

public class PPMyRobot
{
    /* Public OpMode members. */
    public DcMotor  motorFrontLeft   = null;
    public DcMotor  motorFrontRight  = null;
    public DcMotor  motorBackLeft  = null;
    public DcMotor  motorBackRight  = null;
    public DcMotor  motorCarouselSpin  = null;
    public DcMotor  motorArm  = null;
    //public DcMotor  robotArm     = null;
    public Servo    servo_1    = null;
    public Servo    servo_2   = null;
    public Servo    servo_3    = null;

    public static final double MID_SERVO       =  0.5 ;
    public static final double ARM_UP_POWER    =  0.45 ;
    public static final double ARM_DOWN_POWER  = -0.45 ;


    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public PPMyRobot(){

    }


    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        motorFrontLeft = hwMap.dcMotor.get("motorFrontLeft");
        motorFrontRight = hwMap.dcMotor.get("motorFrontRight");
        motorBackLeft = hwMap.dcMotor.get("motorBackLeft");
        motorBackRight = hwMap.dcMotor.get("motorBackRight");
        //motorArm    = hwMap.dcMotor.get("motorArm");
        //motorCarouselSpin = hwMap.dcMotor.get("motorCarousel");
        //servo_1 = hwMap.servo.get("left_hand");
        //servo_2 = hwMap.servo.get("right_hand");
        //servo_3 = hwMap.servo.get(Servo.class, "thumb");


        // Reverse the right side motors
        motorFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);

        // Set all motors to zero power
        motorFrontLeft.setPower(0);
        motorFrontRight.setPower(0);
        motorBackLeft.setPower(0);
        motorBackRight.setPower(0);
        //motorArm.setPower(0);
      //  motorCarouselSpin.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //motorArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Define and initialize ALL installed servos.
        //leftClaw  = hwMap.servo.get("leftHand");
        //rightClaw = hwMap.servo.get("rightHand");
        //leftClaw.setPosition(MID_SERVO);
        //rightClaw.setPosition(MID_SERVO);
    }
}

