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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;

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

@TeleOp(name="Test Arm", group="Pushbot")
// @Disabled
public class testMotorArm extends LinearOpMode {

    DcMotor armMotor;
    AnalogInput potMeter;

    final double armUpperStop = 2.7;
    final double armLowerStop = 0.7;
    double armPower;
    double level;

    @Override
    public void runOpMode() {

        armMotor = hardwareMap.dcMotor.get("armMotor");
        potMeter = hardwareMap.analogInput.get("potMeter");


        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            armPower=gamepad2.left_stick_y/3;

            if ((potMeter.getVoltage() < armLowerStop && armPower > 0) ||
                    (potMeter.getVoltage() > armUpperStop && armPower < 0)) {
                armMotor.setPower(0);
            } else {
                armMotor.setPower(armPower);
            }

            double upperValue=1.5;
            double lowerValue=0.327;

            if(level == 1) { upperValue = 1.948; lowerValue = 1.881; }
            else if(level == 2) { upperValue = 1.56; lowerValue = 1.387; }
            else if(level == 3) { upperValue = 1.18; lowerValue = 0.985; }
            else {return;}

            while (potMeter.getVoltage() > upperValue || potMeter.getVoltage() < lowerValue)
            {
                if (potMeter.getVoltage() > upperValue)
                {
                    armMotor.setPower(0.55);
                }
                else if (potMeter.getVoltage() < lowerValue)
                {
                    armMotor.setPower(-0.55);
                }
            }
            armMotor.setPower(0);

            telemetry.addData("# ARM Power:", armPower);
            telemetry.addData("Pot value", potMeter.getVoltage());
            telemetry.addData("Y value", gamepad2.left_stick_y);










            telemetry.update();
        }
    }
}
