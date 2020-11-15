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

package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

<<<<<<< Updated upstream
@TeleOp(name="Basic: Iterative OpMode", group="Iterative Opmode")
public class SampleOpMode extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
//    private DcMotor leftDrive = null;
//    private DcMotor rightDrive = null;
//
//    /*
//     * Code to run ONCE when the driver hits INIT
=======
    private DcMotor testMotor;
    private DcMotor testMotor2;
//    private Servo testServo;
//    private double max = 0.00; // Maximum rotational position
//    private double min = 1.00; // Minimum rotational position
//    private double currentPos = 0.00;
>>>>>>> Stashed changes

    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
//        leftDrive  = hardwareMap.get(DcMotor.class, "lfmotor");
//        rightDrive = hardwareMap.get(DcMotor.class, "rfmotor");

<<<<<<< Updated upstream
        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
//        leftDrive.setDirection(DcMotor.Direction.FORWARD);
//        rightDrive.setDirection(DcMotor.Direction.REVERSE);
=======
        testMotor = hardwareMap.get(DcMotor.class, "testmotor");
        testMotor2 = hardwareMap.get(DcMotor.class, "testmotor2");
//        testServo = hardwareMap.get(Servo.class, "testservo");
>>>>>>> Stashed changes

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

<<<<<<< Updated upstream
    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        // Setup a variable for each drive wheel to save power level for telemetry
//        double leftPower;
//        double rightPower;
=======
            // gamepad 1
            if(gamepad2.right_bumper) {
                testMotor.setPower(1);
                testMotor2.setPower(.35);
            } else {
                testMotor.setPower(0);
                testMotor2.setPower(0);
            }

//            testMotor.setPower( gamepad1.left_stick_y );
//            testMotor2.setPower( 0.9*gamepad1.left_stick_y );
>>>>>>> Stashed changes

        // Choose to drive using either Tank Mode, or POV Mode
        // Comment out the method that's not used.  The default below is POV.

<<<<<<< Updated upstream
        // POV Mode uses left stick to go forward, and right stick to turn.
        // - This uses basic math to combine motions and is easier to drive straight.
//        double turn  =  gamepad1.right_stick_x;
//        double drive = -gamepad1.left_stick_y;
//        leftPower    = Range.clip(drive + turn, -1.0, 1.0) ;
//        rightPower   = Range.clip(drive - turn, -1.0, 1.0) ;

        // Tank Mode uses one stick to control each wheel.
        // - This requires no math, but it is hard to drive forward slowly and keep straight.
        // leftPower  = -gamepad1.left_stick_y ;
        // rightPower = -gamepad1.right_stick_y ;

//        // Send calculated power to wheels
//        leftDrive.setPower(leftPower);
//        rightDrive.setPower(rightPower);
=======
//            if (gamepad1.right_stick_y != 0) { //if stick is being moved
//                currentPos = Math.abs(gamepad1.right_stick_y);
//                if (gamepad1.a) { max = currentPos; }
//                if (gamepad1.b) { min = currentPos; }
//            } else { //if stick is untouched
//                if (gamepad1.a) { currentPos = max; }
//                if (gamepad1.b) { currentPos = min; }
//            }

//            testServo.setPosition(currentPos);

            telemetry.addData("Motor Power", gamepad1.left_stick_y);
            telemetry.addData("Right Stick Pos", gamepad1.right_stick_y);
//            telemetry.addData("Servo Position", currentPos);
//            telemetry.addData("Servo Max", max);
//            telemetry.addData("Servo Min", min);
            telemetry.update();
>>>>>>> Stashed changes

        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
//        telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
