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
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/**
 * This file provides basic Telop driving for a Pushbot robot.
 * The code is structured as an Iterative OpMode
 *
 * This OpMode uses the common Pushbot hardware class to define the devices on the robot.
 * All device access is managed through the HardwarePushbot class.
 *
 * This particular OpMode executes a basic Tank Drive Teleop for a PushBot
 * It raises and lowers the claw using the Gampad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */
@Disabled

@TeleOp(name="Basic Driving", group="Pushbot")
public class BasicDriving extends OpMode {

    /* Declare OpMode members. */
    HardwarePushbotBurg robot = new HardwarePushbotBurg();
    private boolean isHolding = false;
    private boolean isOpening = false;
    private boolean hasRun = false;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);
        robot.initIMU(hardwareMap);
        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");

        robot.gripper.setPosition(0);
        pauseForATime(1);
        robot.gripper.setPosition(0);
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
        public void pauseForATime(double timeDelay) {
            if (getRuntime() > time + timeDelay) {
                time = getRuntime();
            }
        }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        boolean dpadUp;
        final double MAX_POWER = .8;
        boolean dpadDown;
        boolean dpadDown2;
        boolean dpadUp2;
        boolean a_button;
        boolean b_button;
        boolean a_button2;
        boolean b_button2;
        boolean sens_clutch;
        float right_trig;
        float left_trig;
        boolean left_bump;
        boolean right_bump;

        double i;
        double l = 0;
        double open = 0.1;
        // Run wheels in tank mode (note: The joystick goes negative when pushed forwards, so negate it)
        double forward = -gamepad1.left_stick_y;
        double turn = -gamepad1.right_stick_x;
        dpadDown = gamepad1.dpad_down;
        dpadUp = gamepad1.dpad_up;
        a_button = gamepad1.a;
        b_button = gamepad1.b;
        sens_clutch = gamepad1.right_bumper;
        right_trig = gamepad1.right_trigger;
        left_trig = gamepad1.left_trigger;

        right_bump = gamepad1.right_bumper;
        left_bump = gamepad1.left_bumper;

        a_button2 = gamepad2.a;
        b_button2 = gamepad2.b;
        dpadDown2 = gamepad2.dpad_down;
        dpadUp2 = gamepad2.dpad_up;

        double leftPower = forward - turn;
        double rightPower = forward + turn;

        double fullPower = MAX_POWER;
        double maxInput = Math.max(Math.abs(leftPower), Math.abs(rightPower));
        if (maxInput > fullPower) {
            double scale = fullPower / maxInput;
            leftPower = scalePower(scale, leftPower, true);
            rightPower = scalePower(scale, rightPower, true);
        }
        if(sens_clutch){
            leftPower = leftPower/2;
            rightPower = rightPower/2;
        }

        telemetry.addData("LeftPower:", leftPower);
        telemetry.addData("RightPower:", rightPower);
        telemetry.addData("Clutch:", sens_clutch);

        robot.frontLeft.setPower(leftPower);
        robot.backLeft.setPower(leftPower);
        robot.frontRight.setPower(rightPower);
        robot.backRight.setPower(rightPower);

        if (dpadUp) {
            l = 1;
        } else if (dpadDown) {
            l = -1;
        } else {
            if (dpadUp2) {
                l = 1;
            } else if (dpadDown2) {
                l = -1;
            } else {
                l = 0;
            }
        }







        //We don't want it to hit the bottom or top too hard

      /*  if (robot.sensorRange.getDistance(DistanceUnit.INCH) < robot.MIN_HEIGHT) {
            l = 0.35;
            telemetry.addData("Moving up", "");
        } else if (robot.sensorRange.getDistance(DistanceUnit.INCH) > robot.MAX_HEIGHT) {
           // l = -0.35;
            telemetry.addData("Moving down", "");
        }
*/
        robot.lift.setPower(l);


        if (a_button) {
          //  if (robot.gripper.getPosition() < 0.2) {
                robot.gripper.setPosition(1);
          //  } else
          //     robot.gripper.setPosition(open);
          //  }
        }
        if (b_button2) {
            robot.gripper.setPosition(open);
        }

        if (a_button2) {
            //  if (robot.gripper.getPosition() < 0.2) {
            robot.gripper.setPosition(1);
            //  } else
            //     robot.gripper.setPosition(open);
            //  }
        }
        if (b_button) {
            robot.gripper.setPosition(open);
        }

        telemetry.addData("servo position", robot.gripper.getPosition());
 //       telemetry.addData("range", String.format("%.01f in", robot.sensorRange.getDistance(DistanceUnit.INCH)));
        }

    private double scalePower(double scale,double input,boolean curveThePower){
        // Design a power curve so that full speed (input: 1) is still 1, but
        // small inputs are made even smaller
        if (curveThePower) {
            telemetry.addData("curving power!", "");
            double curvedPower = (scale * input) * (scale * input);
            if (input < 0) curvedPower = -curvedPower;
            return curvedPower;
        }
        return scale * input;
    }


    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
}
