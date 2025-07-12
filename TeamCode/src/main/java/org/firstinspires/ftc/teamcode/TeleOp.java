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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

/*
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="teleop", group="Iterative OpMode")
public class TeleOp extends OpMode
{
    // Declare OpMode members.


    Robot robot = new Robot();

    @Override
    public void init() {
        robot.init(hardwareMap);
//        robot.arm.wrist.setPosition(0);
    }


    @Override
    public void loop() {
        robot.db.drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        robot.arm.updateArm();
//        robot.ll.updateLift();
        if (gamepad2.x) {
            robot.arm.setPosition(0);
        } else if (gamepad2.b) {
            robot.arm.setPosition(-1000);
        } else if (gamepad2.y){
            robot.arm.setPosition(-700);
        }
//        if (gamepad2.a) {
//            robot.arm.arm.setPower(1);
//        } else if (gamepad2.b) {
//            robot.arm.arm.setPower(-1);
//        } else {
//            robot.arm.arm.setPower(0);
//        }

//        else if (gamepad2.right_stick_y > .1) {
//            robot.arm.setPosition(-750);
//        }

        if (gamepad2.right_trigger > .1) {
            robot.arm.spinIn();
        } else if (gamepad2.left_trigger > .1){
            robot.arm.spinOut();
        } else {
            robot.arm.spinOff();
        }
//
//        if(gamepad2.dpad_up) {
//            robot.ll.setLift(0);
//        } else if (gamepad2.dpad_left) {
//            robot.ll.setLift(-1000);
//        } else if (gamepad2.dpad_down) {
//            robot.ll.setLift(-1500);
//        } else if (gamepad2.dpad_right) {
//            robot.ll.setLift(-2000);
//        }
//        telemetry.addData("Arm pos", robot.ll.lift.getCurrentPosition());

//        if(gamepad2.dpad_up) {
//            robot.ll.lift.setPower(.5);
//        } else if (gamepad2.dpad_down) {
//            robot.ll.lift.setPower(-.5);
//        }else {
//            robot.ll.lift.setPower(0);
//        }

////        robot.ll.setLift(100);
//        telemetry.addData("liftTargetPosition", robot.ll.targetPosition);
//        telemetry.addData("lifterror", robot.ll.targetPosition - robot.ll.lift.getCurrentPosition());
//        telemetry.addData("liftCurrentPosition", robot.ll.lift.getCurrentPosition());
//        telemetry.addData("liftCurrentPower", robot.ll.lift.getPower());
        telemetry.addData("armTargetPosition", robot.arm.targetPosition);
        telemetry.addData("error", robot.arm.targetPosition - robot.arm.arm.getCurrentPosition());
        telemetry.addData("armCurrentPosition", robot.arm.arm.getCurrentPosition());
        telemetry.addData("armCurrentPower", robot.arm.arm.getPower());
        telemetry.addData("intake power", robot.arm.hand.getPower());
    }

}
