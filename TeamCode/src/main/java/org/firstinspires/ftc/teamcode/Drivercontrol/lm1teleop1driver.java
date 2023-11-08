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

package org.firstinspires.ftc.teamcode.Drivercontrol;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.extension;


/*
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@TeleOp(name="lm1drive1driver", group="Linear OpMode")

public class lm1teleop1driver extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    Servo wrist;
    Servo claw;

    Feildcentricdrive drive = new Feildcentricdrive();

    @Override
    public void runOpMode() {

        boolean open = false;
        boolean state = false;
        boolean lastState = false;// claw booleans
        wrist = hardwareMap.get(Servo.class, "wrist");
        claw = hardwareMap.get(Servo.class, "claw");//hardwaremap claw and tilt
        extension extend = new extension(hardwareMap);//tilt object made
        extend.release();//releases the tilt to move into start pos
        boolean reset = false;//boolean for reset heading
        boolean slowturn = false;//boolean for slowturn mode
        double rx;//right x
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        drive.init(hardwareMap, 0);//init dt
        //extend.setStowPos();//for when auto is ready
        while(opModeInInit()){
            if(gamepad1.a){
                extend.hold();//hold init pos
                break;
            }
        }
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            if (gamepad1.a) {//slow turn mode
                slowturn = true;
            }
            if (gamepad1.b) {
                slowturn = false;
            }
            rx = gamepad1.right_stick_x;
            if (slowturn) {
                rx *=0.7;
            } //could be more compact

            if (gamepad1.y) {//slow speed mode
                drive.run(gamepad1.left_stick_y, gamepad1.left_stick_x, rx, 0.5, true);
            } else {
                drive.run(gamepad1.left_stick_y, gamepad1.left_stick_x, rx, 1, true, gamepad1.back);
            }

            state = gamepad1.left_bumper;//state for claw

            if (gamepad1.left_trigger >= 0.5) {extend.setIntake();wrist.setPosition(1);}//set posistions
            else if (gamepad1.right_bumper) {extend.setIntake();wrist.setPosition(0.3);}
            else if (gamepad1.right_trigger >= 0.5) {extend.setPlace();wrist.setPosition(1);}

            if (gamepad1.left_bumper && state != lastState) {//new claw code for easier driving
                if (open) {claw.setPosition(0.2);open = false;}
                else {claw.setPosition(1);open = true;}
            }
//            if(gamepad1.dpad_up) {//old claw code
//                claw.setPosition(0.2);
//
//            }else if(gamepad1.dpad_down)
//                claw.setPosition(1);
//
//            }

            lastState = state;//set last state to the state at end of loop
        }
    }
}

