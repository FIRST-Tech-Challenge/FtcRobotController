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

package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.util.Hardware;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.NewHardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import java.util.concurrent.locks.Lock;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
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
 * This particular O;pMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */
@Disabled
@TeleOp(name="TeleOp_Lift", group="Iterative Opmode")

public class TeleOp_Lift extends OpMode
{

    /* Declare OpMode members. */
    NewHardwareMap robot= new NewHardwareMap(); // use the class created to define Robot hardware
    boolean a_pressed = false;
    boolean b_pressed = false;
    boolean x_pressed = false;
    boolean y_pressed = false;
    boolean ShooterOn = false;
    private int power = 0;
    private double powerIncrement = 0.01;
    private double powerMax = 0.6;
    private double powerDown = -0.6;
    private double powerMin = -0.2;
    /*
     * Code to run when the op mode is first enabled goes here
     *
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#start()
     */
    @Override
    public void init() {

        robot.init(hardwareMap);

        // assign the starting position of the wrist and hand
        /*robot.ScoopL.setPosition(robot.ScoopL_Down);
        robot.ScoopR.setPosition(robot.ScoopR_Down);
        robot.Hammer.setPosition(robot.Hammer_stowed);
        robot.WG_lock.setPosition(robot.WG_locked);*/

    }


    /*
     * This method will be called repeatedly in a loop
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#run()
     */
    @Override
    public void loop()
    {
        double power =  gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double turn  =  gamepad1.right_stick_x;
        double left    = Range.clip(power - turn, -0.8, 0.8);
        double right   = Range.clip(power + turn, -0.8, 0.8) ;

        // scale the joystick value to make it easier to control
        // the robot more precisely at slower speeds.
    /*right = (float)scaleInput(right);
    left =  (float)scaleInput(left);

    //Turbo to 100%
    if (gamepad1.left_stick_button){
        right = Range.clip(right * 1.4, -1.0, 1.0);
        left  = Range.clip(left * 1.4, -1.0, 1.0);}*/


//(Note: The joystick goes negative when pushed forwards, so negate it for robot to drive forwards.)
//Left Joystick Manipulates Left Motors
    /*robot.FmotorLeft.setPower(left-strafe);
    robot.BmotorLeft.setPower(left+strafe);
    robot.FmotorRight.setPower(right+strafe);
    robot.BmotorRight.setPower(right-strafe);*/


        //DuckMotor
    /*if (gamepad1.b && !b_pressed){  //Only toggle on leading edge
        robot.DuckMotor.setPower(-0.6);
    }else{
        robot.DuckMotor.setPower(0);
        b_pressed = true;
    }


    // Intake Motor Control
    if (gamepad1.left_bumper){  //Only toggle on leading edge
                robot.Intake1.setPower(1.0);
                robot.Intake2.setPower(1.0);
            }else{
                robot.Intake1.setPower(0);
                robot.Intake2.setPower(0);
            }*/
        //Out
    /*if (gamepad1.left_trigger > 0.25 && !gamepad1.left_bumper) {
           if (gamepad1.a && !a_pressed) {
            robot.Intake1.setPower(-1);
            robot.Intake2.setPower(-1);
            }else{
            robot.Intake1.setPower(-0.75);
            robot.Intake2.setPower(-0.75);
            }
    }else{
            robot.Intake1.setPower(0);
            robot.Intake2.setPower(0);
        }*/

        // Lift Motor Control
        if ((gamepad1.right_bumper) && (!robot.touch3.isPressed())){
            robot.LiftMotor.setPower(1.0);   // Lift UP
        }else if ((gamepad1.right_trigger) > 0.25 && (!robot.touch.isPressed())) {
            robot.LiftMotor.setPower(-0.8);  // Lift DOWN
        }else{
            robot.LiftMotor.setPower(0.0);
            powerDown = 0;
        }

      /*if (gamepad1.x && !x_pressed) {
          robot.LiftMotor.setPower(-0.6);
      }
      else{
          robot.LiftMotor.setPower(0);
      }*/

        //Reset button toggles
        if (!gamepad1.a) a_pressed = false;
        if (!gamepad1.b) b_pressed = false;
        if (!gamepad1.x) x_pressed = false;
        if (!gamepad1.y) y_pressed = false;


    }//loop end


    // Code to Run When Coach Hits STOP
    @Override
    public void stop()
    {
        telemetry.addData("Robot", "Stopped");
    }

    /*
     * This method scales the joystick input so for low joystick values, the
     * scaled value is less than linear.  This is to make it easier to drive
     * the robot more precisely at slower speeds.
     */
    double scaleInput(double dVal)  {
        double[] scaleArray = { 0.0, 0.05, 0.09, 0.10, 0.12, 0.15, 0.18, 0.20,
                //0.30, 0.40, 0.50, 0.55, 0.60, 0.65, 0.7, 0.75, 0.75};
                0.30, 0.36, 0.43, 0.50, 0.60, 0.72, 0.85, 1.00, 1.00 };
        // get the corresponding index for the scaleInput array.
        int index = (int) (dVal * 16.0);
        if (index < 0) {
            index = -index;
        } else if (index > 16) {
            index = 16;
        }
        double dScale = 0.0;
        if (dVal < 0) {
            dScale = -scaleArray[index];
        } else {
            dScale = scaleArray[index];
        }
        return dScale;
    }
}