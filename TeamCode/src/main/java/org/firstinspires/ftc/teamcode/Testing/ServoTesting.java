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

package org.firstinspires.ftc.teamcode.Testing;

import android.widget.Button;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.GoBuilda.GoBuildaDriveToPointExample;
import org.firstinspires.ftc.teamcode.Robot.ScaledServo;
import org.firstinspires.ftc.teamcode.Robot.TT_RobotHardware;

@TeleOp(name = "Servo Test", group = "Testing")

public class ServoTesting extends LinearOpMode {

    static final double INCREMENT = 0.05;     // amount to slew servo each CYCLE_MS cycle
    static final int CYCLE_MS = 50;     // period of each cycle

    // Define class members
    public Servo servo;
    public Servo extension = null;
    public Servo extensionArm = null;
    public Servo extensionSpin = null;
    public Servo liftArm = null;
    public Servo light = null;
    public Servo test = null;
    public Servo gripper = null;
    //DigitalChannel button;
    double servo_position;

    enum StateMachine {
        WAITING_FOR_START,
        LIFT_ARM,
        EXTENSION,
        EXTENSION_ARM,
        EXTENSION_SPIN,
        GRIPPER,
        TEST,
        BUTTON
    }
    boolean dpadPressed = false;
    boolean abPressed = false;

    @Override
    public void runOpMode() {
        StateMachine stateMachine;
        stateMachine = StateMachine.LIFT_ARM;
        extension = hardwareMap.get(Servo.class, "Extension");
        extensionArm = hardwareMap.get(Servo.class, "Extension Arm");
        extensionSpin = hardwareMap.get(Servo.class, "Extension Spin");
        liftArm = hardwareMap.get(Servo.class, "Lift Arm");
        light = hardwareMap.get(Servo.class, "Light");
        gripper = hardwareMap.get(Servo.class, "Extension Gripper");
        test = hardwareMap.get(Servo.class, "test");

        servo = liftArm;
        servo_position = servo.getPosition();

        telemetry.addData("State", stateMachine.toString());
        telemetry.addData("Servo Position", "%5.2f", liftArm.getPosition());
        telemetry.addData("Servo Target", "%5.2f", servo_position);
        //telemetry.addData("Button Pressed", "%s", button.getState());
        telemetry.addData(">", "Press Start to scan Servo.");
        telemetry.update();
        // Wait for the start button
        waitForStart();

        // Scan servo till stop pressed.
        while (opModeIsActive()) {
            if (gamepad1.dpad_up) {
                if (!dpadPressed) {
                    switch (stateMachine) {
                        case LIFT_ARM:
                            stateMachine = StateMachine.EXTENSION;
                            servo = extension;
                            servo_position = extension.getPosition();
                            break;
                        case EXTENSION:
                            stateMachine = StateMachine.EXTENSION_ARM;
                            servo = extensionArm;
                            servo_position = extensionArm.getPosition();
                            break;
                        case EXTENSION_ARM:
                            stateMachine = StateMachine.EXTENSION_SPIN;
                            servo = extensionSpin;
                            servo_position = extensionSpin.getPosition();
                            break;
                        case EXTENSION_SPIN:
                            stateMachine = StateMachine.GRIPPER;
                            servo = gripper;
                            servo_position = gripper.getPosition();
                            break;
                        case GRIPPER:
                            stateMachine = StateMachine.TEST;
                            servo = test;
                            servo_position = test.getPosition();
                            break;
                        case TEST:
                            stateMachine = StateMachine.LIFT_ARM;
                            servo = liftArm;
                            servo_position = liftArm.getPosition();
                            break;
                    }
                }
                dpadPressed = true;
            } else {
                if (!abPressed) {
                    //if (!button.getState() || button.getState()) {
                    // slew the servo, according to the rampUp (direction) variable.
                    if (gamepad1.a) {
                        // Keep stepping up until we hit the max value.
                        servo_position = servo.getPosition() + INCREMENT;
                        abPressed = true;
                    } else if (gamepad1.b) {
                        // Keep stepping down until we hit the min value.
                        servo_position = servo.getPosition() - INCREMENT;
                        abPressed = true;
                    }
                }
                else {
                    abPressed = false;
                }
                dpadPressed = false;
            }
            servo.setPosition(servo_position);

            // Display the current value

            telemetry.addData("State", stateMachine.toString());
            telemetry.addData("Servo Position", "%5.2f", servo.getPosition());
            telemetry.addData("Servo Target", "%5.2f", servo_position);
            //telemetry.addData("Button Pressed", "%s", button.getState());
            telemetry.addData(">", "Press Stop to end test.");
            telemetry.update();
            sleep(CYCLE_MS);
            idle();
        }

    }
}
