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

import android.graphics.Color;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Mr. Price's teleOp for test and explanation
 *
 * Need to confirm the drive system and add the drone servo to have a full test case
 * Also need the telemetry to read all sensor values
 *
 * I2C Bus 1: Port 1: FDS Rev 2m Distance sensor:  distFront
 * Port2 RDS: Rev 2m Distance sensor: distRear
 * See the sensor's product page: https://www.revrobotics.com/rev-31-1505/
 */

@TeleOp(name="coachArmTest", group="TeleOp")

public class coachNewArm extends LinearOpMode {

    /* Declare OpMode members. */
    public DcMotor leftArm;
    public DcMotor rightArm;

    public Servo drone;

    double clawOffset = 0;

    /*public static final double MID_SERVO   =  0.5 ;
    public static final double CLAW_SPEED  = 0.02 ;                 // sets rate to move servo
    public static final double ARM_UP_POWER    =  0.45 ;
    public static final double ARM_DOWN_POWER  = -0.45 ;
    */

    public void runOpMode() throws InterruptedException {
        double driveY = 0;
        double strafe = 0;
        double turn = 0;
        double RWPower = 0;
        double LWPower = 0;
        double drivePower = 0.5; //global drive power level
        double armPower = 0;
        double droneSet = 0.7;
        double droneLaunch = 0.4;

        // Define and Initialize Motors
        leftArm = hardwareMap.get(DcMotor.class, "armL");
        rightArm = hardwareMap.get(DcMotor.class, "armR");

        drone.setPosition(droneSet);

        // Send telemetry message to signify robot waiting;
        telemetry.addData(">", "Robot Ready.  Press Play.");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        if (isStopRequested()) return;
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            while (opModeIsActive()) {
                armPower = gamepad2.left_stick_y;

                leftArm.setPower(armPower);
                rightArm.setPower(armPower);



                telemetry.addData(">", "Robot Running");
                telemetry.addData("Arm Power", armPower);
                telemetry.addData("Drone Servo"), 
                telemetry.update();

            }
        }
    }
}
