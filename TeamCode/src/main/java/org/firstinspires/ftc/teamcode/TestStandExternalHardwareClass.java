package org.firstinspires.ftc.teamcode;/* Copyright (c) 2022 FIRST. All rights reserved.
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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/*
 * This OpMode illustrates how to use an external "hardware" class to modularize all the robot's sensors and actuators.
 * This approach is very efficient because the same hardware class can be used by all of your teleop and autonomous OpModes
 * without requiring many copy & paste operations.  Once you have defined and tested the hardware class with one OpMode,
 * it is instantly available to other OpModes.
 *
 * The real benefit of this approach is that as you tweak your robot hardware, you only need to make changes in ONE place (the Hardware Class).
 * So, to be effective you should put as much or your hardware setup and access code as possible in the hardware class.
 * Essentially anything you do with hardware in BOTH Teleop and Auto should likely go in the hardware class.
 *
 * The Hardware Class is created in a separate file, and then an "instance" of this class is created in each OpMode.
 * In order for the class to do typical OpMode things (like send telemetry data) it must be passed a reference to the
 * OpMode object when it's created, so it can access all core OpMode functions.  This is illustrated below.
 *
 * In this concept sample, the hardware class file is called RobotHardware.java and it must accompany this sample OpMode.
 * So, if you copy ConceptExternalHardwareClass.java into TeamCode (using Android Studio or OnBotJava) then RobotHardware.java
 * must also be copied to the same location (maintaining its name).
 *
 * For comparison purposes, this sample and its accompanying hardware class duplicates the functionality of the
 * RobotTelopPOV_Linear OpMode.  It assumes three motors (left_drive, right_drive and arm) and two servos (left_hand and right_hand)
 *
 * View the RobotHardware.java class file for more details
 *
 *  Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 *  Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 *
 *  In OnBot Java, add a new OpMode, select this sample, and select TeleOp.
 *  Also add another new file named RobotHardware.java, select the sample with that name, and select Not an OpMode.
 */

@TeleOp(name="Concept: Test Stand Hardware Class", group="robot")
// @Disabled
public class TestStandExternalHardwareClass extends LinearOpMode {

    /// Define Motor and Servo objects  (Make them private so they can't be accessed externally)
    private DcMotor motor117    = null;
    private DcMotor motor312    = null;
    private DcMotor motor1150   = null;
    private DcMotor leftPiston  = null;
    private DcMotor rightPiston = null;


    @Override
    public void runOpMode() {

        // Define and Initialize Motors (note: need to use reference to actual OpMode).
        motor117    = this.hardwareMap.get(DcMotor.class, "motor117");
        motor312    = this.hardwareMap.get(DcMotor.class, "motor312");
        motor1150   = this.hardwareMap.get(DcMotor.class, "motor1150");
        leftPiston  = this.hardwareMap.get(DcMotor.class, "leftPiston");
        rightPiston = this.hardwareMap.get(DcMotor.class, "rightPiston");

        // Send telemetry message to signify robot waiting;
        // Wait for the game to start (driver presses START)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Read pad inputs and adjust motor speeds.
            motor117.setPower(gamepad1.left_stick_y);
            motor312.setPower(gamepad1.left_stick_x);
            motor1150.setPower(gamepad1.right_stick_y);
            leftPiston.setPower(gamepad2.left_stick_y);
            rightPiston.setPower(gamepad2.right_stick_y);

            /*

            // Combine drive and turn for blended motion. Use RobotHardware class
            robot.driveRobot(drive, turn);

            // Use gamepad left & right Bumpers to open and close the claw
            // Use the SERVO constants defined in RobotHardware class.
            // Each time around the loop, the servos will move by a small amount.
            // Limit the total offset to half of the full travel range
            if (gamepad1.right_bumper)
                handOffset += robot.HAND_SPEED;
            else if (gamepad1.left_bumper)
                handOffset -= robot.HAND_SPEED;
            handOffset = Range.clip(handOffset, -0.5, 0.5);

            // Move both servos to new position.  Use RobotHardware class
            robot.setHandPositions(handOffset);

            // Use gamepad buttons to move arm up (Y) and down (A)
            // Use the MOTOR constants defined in RobotHardware class.
            if (gamepad1.y)
                arm = robot.ARM_UP_POWER;
            else if (gamepad1.a)
                arm = robot.ARM_DOWN_POWER;
            else
                arm = 0;

            robot.setArmPower(arm);

             */

            /*
            // Send telemetry messages to explain controls and show robot status
            telemetry.addData("Power_117", "Pad 1/Left Stick - Up/Down");
            telemetry.addData("Power_312", "Pad 1/Left Stick - Side to side");
            telemetry.addData("Power_1150", "Pad 1/Right Stick - Up/Down");
            telemetry.addData("Power_Left_Piston", "Pad 2/Left Stick - Up/Down");
            telemetry.addData("Power_Right_Piston", "Pad 2/Right Stick - Up/Down");
            telemetry.addData("-", "-------");

            telemetry.addData("117 Power", "%.2f", motor117);
            telemetry.addData("312 Power",  "%.2f", motor312);
            telemetry.addData("1150 Power",  "%.2f", motor1150);
            telemetry.addData("Power_Left_Piston",  "%.2f", leftPiston);
            telemetry.addData("Power_Right_Piston",  "%.2f", rightPiston);
            telemetry.update();
            -
             */

            // Pace this loop so hands move at a reasonable speed.
            sleep(50);
        }
    }
}
