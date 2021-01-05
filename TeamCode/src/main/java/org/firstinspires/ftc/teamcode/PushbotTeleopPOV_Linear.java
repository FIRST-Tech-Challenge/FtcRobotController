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
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;
import org.firstinspires.ftc.teamcode.HardwareBaymax;
//import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;

/**
 * This OpMode uses the common Pushbot hardware class to define the devices on the robot.
 * All device access is managed through the HardwarePushbot class.
 * The code is structured as a LinearOpMode
 *
 * This particular OpMode executes a POV Game style Teleop for a PushBot
 * In this mode the left stick moves the robot FWD and back, the Right stick turns left and right.
 * It raises and lowers the claw using the Gampad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Pushbot: Teleop POV Coach 20210104", group="Pushbot")
//@Disabled
public class PushbotTeleopPOV_Linear extends LinearOpMode {

    /* Declare OpMode members. */
    HardwareBaymax robot           = new HardwareBaymax();   // Use a Pushbot's hardware
    double          clawOffset      = 0;                       // Servo mid position
    final double    CLAW_SPEED      = 0.02 ;                   // sets rate to move servo

    @Override
    public void runOpMode() {
        double leftfront;
        double rightfront;
        double leftback;
        double rightback;
        double drive;
        double turn;
        double strife;
        double max;

        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Howdy partner, good on ya katie rules");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Run wheels in POV mode (note: The joystick goes negative when pushed forwards, so negate it)
            // In this mode the Left stick moves the robot fwd and back, the Right stick turns leftfront and rightfront.
            // This way it's also easy to just drive straight, or just turn.
            drive = gamepad1.left_stick_y;
            turn  =  -gamepad1.right_stick_x;
            strife = gamepad1.left_stick_x;

            // Combine drive and turn for blended motion.
            if (strife == 0)
            {
                leftfront = drive + turn;
                rightfront = drive - turn;
                leftback = drive + turn;
                rightback = drive - turn;
            }
            else
            {
                leftfront = -strife;
                rightfront = strife;
                leftback = strife;
                rightback = -strife;
            }

            // Normalize the values so neither exceed +/- 1.0
            max = Math.max(Math.abs(leftfront), Math.abs(rightfront));
            if (max > 1.0)
            {
                leftfront /= max;
                rightfront /= max;
                leftback /= max;
                rightback /= max;
            }

            // Output the safe vales to the motor drives.
            robot.leftfrontDrive.setPower(leftfront*0.5);
            robot.rightfrontDrive.setPower(rightfront*0.5);
            robot.leftbackDrive.setPower(leftback*0.5);
            robot.rightbackDrive.setPower(rightback*0.5);


            // Use gamepad leftfront & rightfront Bumpers to open and close the claw
//            if (gamepad1.right_bumper)
//                clawOffset += CLAW_SPEED;
//            else if (gamepad1.left_bumper)
//                clawOffset -= CLAW_SPEED;

            // Move both servos to new position.  Assume servos are mirror image of each other.
//            clawOffset = Range.clip(clawOffset, -0.5, 0.5);
//            robot.leftClaw.setPosition(robot.MID_SERVO + clawOffset);
//            robot.rightClaw.setPosition(robot.MID_SERVO - clawOffset);

            // Use gamepad buttons to move arm up (Y) and down (A)
//            if (gamepad1.y)
//                robot.leftArm.setPower(robot.ARM_UP_POWER);
//            else if (gamepad1.a)
//                robot.leftArm.setPower(robot.ARM_DOWN_POWER);
//            else
//                robot.leftArm.setPower(0.0);

            // Send telemetry message to signify robot running;
//            telemetry.addData("claw",  "Offset = %.2f", clawOffset);
            telemetry.addData("leftfront",  "%.2f", leftfront);
            telemetry.addData("rightfront", "%.2f", rightfront);

            telemetry.addData("leftback",  "%.2f", leftback);
            telemetry.addData("rightback", "%.2f", rightback);
            telemetry.update();

            // Pace this loop so jaw action is reasonable speed.
            sleep(50);
        }
    }
}
