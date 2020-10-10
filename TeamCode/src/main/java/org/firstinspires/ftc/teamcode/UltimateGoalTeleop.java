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

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import static org.firstinspires.ftc.teamcode.HardwareUltimateGoal.TeleOpRunMode;


/**
 * This file provides basic Telop driving for a RoverRuckus robot.
 * The code is structured as an Iterative OpMode
 *
 * This OpMode uses the common RoverRuckus hardware class to define the devices on the robot.
 * All device access is managed through the HardwareRoverRuckus class.
 *
 */

@TeleOp(name="Ultimate Goal Teleop", group="UltimateGoal")
public class UltimateGoalTeleop extends OpMode{


    // declaring variables
    MecanumDriveTrain vroom;

    /* Declare OpMode members. */
    HardwareMapV2 robot; // use the class created to define a RoverRuckus's hardware

    teleConfigTest t;
    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */

        robot.init();
        robot.setEncoders(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // initializing the variables


        vroom = new MecanumDriveTrain(robot, gamepad1,telemetry);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Haddi", "Haddi");
        telemetry.update();
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {

        //Mecanum Drivetrain function to set powers
        vroom.loop();

        if (gamepad2.a) {
            robot.intake.setPower((robot.intake.getPower() >= 0.1) ? 0 : 1);
        }
        if(gamepad2.b){
            robot.outtake.setPower((robot.outtake.getPower() >= 0.1) ? 0 : 1);
            robot.conveyor.setPower((robot.conveyor.getPower() >=0.1)? 0:1);
        }
        if(gamepad2.right_stick_y!=0){
            robot.leftTilt.setPower(gamepad2.right_stick_y);
            robot.rightTilt.setPower(gamepad2.right_stick_y);
        }
        if (gamepad2.a){t.a();}
        if(gamepad2.b){t.b();}
        if (gamepad2.x){t.x();}
        if (gamepad2.y){t.y();}

        t.rjoy(gamepad2.right_stick_x, gamepad2.right_stick_y);
        t.ljoy(gamepad2.left_stick_x, gamepad2.left_stick_y);


    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
}