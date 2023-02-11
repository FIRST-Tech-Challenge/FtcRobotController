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
import java.lang.annotation.Target;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import java.util.Date;
import org.firstinspires.ftc.robotcore.external.State;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.FRONT;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.Vector;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

//import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.RobotLog;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import java.util.concurrent.locks.Lock;
import com.qualcomm.hardware.bosch.BNO055IMU;

import java.lang.Math.*;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.teamcode.NewHardwareMap;


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

@TeleOp(name="Power_TeleOp", group="Iterative Opmode")

public class Power_TeleOp extends OpMode
{

    /* Declare OpMode members. */
    NewHardwareMap robot= new NewHardwareMap(); // use the class created to define Robot hardware
    boolean a_pressed = false;
    boolean b_pressed = false;
    boolean x_pressed = false;
    boolean y_pressed = false;
    boolean bumper_pressed = false;
    boolean ShooterOn = false;
    static final double     PI = Math.PI;
    static final double     COUNTS_PER_MOTOR_REV    = 537.6;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0;     // This is < 1.0 if geared UP 0.025
    static final double     WHEEL_DIAMETER_INCHES   = 3.78;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * PI);
    static final double LiftPos = 0;
    static final double posOpen = 0.5;
    static final double posClose =0.9;

    //double wPower = 0.0;
    /*
     * Code to run when the op mode is first enabled goes here
     *
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#start()
     */

    @Override

    public void init() {
        RobotLog.d("LOGGING START");
        robot.init(hardwareMap);


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
        double left    = Range.clip(power - turn, -0.7, 0.7);
        double right   = Range.clip(power + turn, -0.7, 0.7) ;

        // scale the joystick value to make it easier to control
        // the robot more precisely at slower speeds.
        right = (float)scaleInput(right);
        left =  (float)scaleInput(left);

        //Turbo to 100%
        if (gamepad1.x && !x_pressed){
            right = Range.clip(right * 1.4, -1.0, 1.0);
            left  = Range.clip(left * 1.4, -1.0, 1.0);}


//(Note: The joystick goes negative when pushed forwards, so negate it for robot to drive forwards.)
//Left Joystick Manipulates Left Motors
        robot.FmotorLeft.setPower(left-strafe);
        robot.BmotorLeft.setPower(left+strafe);
        robot.FmotorRight.setPower(right+strafe);
        robot.BmotorRight.setPower(right-strafe);


        //Claw intake code
        //Closed
    /*if (gamepad1.left_bumper && !bumper_pressed) {
        if (robot.FreightArm.getPosition()<(0.7)){  //Only toggle on leading edge
                robot.FreightArm.setPosition(0.9);
            } //Opened
            else{//if (robot.FreightArm.getPosition()>(0.7)){  //Only toggle on leading edge
                robot.FreightArm.setPosition(0.5);
            }
            bumper_pressed = true;
    }*/



        if (gamepad1.left_bumper){  //Only toggle on leading edge //Intake in
            robot.Gray.setPower(1);
            robot.Green.setPower(-1);
        }else{
            if ((gamepad1.left_trigger) > 0.25){  //Only toggle on leading edge //Intake out                                                                                                                     :)
                robot.Gray.setPower(-1);
                robot.Green.setPower(1);
            }else{
                robot.Gray.setPower(0);
                robot.Green.setPower(0);
            }
        }

        // Lift Motor Control
        if ((gamepad1.right_bumper) && (gamepad1.right_trigger) > 0.25 && (!robot.touch.isPressed())) {
            robot.LiftMotor.setPower(-1.0);
        }
        else if ((gamepad1.right_bumper) && (!robot.touch3.isPressed()) && (gamepad1.right_trigger) < 0.25){
            robot.LiftMotor.setPower(1.0);   // Lift UP
        }else if ((gamepad1.right_trigger) > 0.25 && (!robot.touch.isPressed()) && (!gamepad1.right_bumper)) {
            robot.LiftMotor.setPower(-0.5);  // Lift DOWN
        }/*else if (gamepad1.x && !x_pressed && (!robot.touch.isPressed())){  //Only toggle on leading edge

            robot.LiftMotor.setPower(-1.0);

        }*/else{
            robot.LiftMotor.setPower(0.0);
            //x_pressed = true;
        }

        /*if(gamepad1.y && !y_pressed) {
                robot.LiftMotor.setPower(-1.0);
        }

        if (robot.touch.isPressed()) {
            robot.LiftMotor.setPower(0);
        }*/

        //Reset button toggles
        //if (!gamepad1.a) a_pressed = false;
        if (!gamepad1.b) b_pressed = false;
        if (!gamepad1.x) x_pressed = false;
        if (!gamepad1.y) y_pressed = false;
        if (!gamepad1.left_bumper) bumper_pressed = false;

        //if (!gamepad2.a) a_pressed = false;
        //if (!gamepad2.b) b_pressed = false;
        //if (!gamepad2.x) x_pressed = false;
        //if (!gamepad2.y) y_pressed = false;

        //telemetry.addData("range", String.format("%.01f cm", robot.sensorRange.getDistance(DistanceUnit.CM)));
        //telemetry.addData("range", String.format("%.01f in", robot.sensorRange.getDistance(DistanceUnit.INCH)));
        //RobotLog.d("%.01f cm,%.01f in,",robot.sensorRange.getDistance(DistanceUnit.CM),robot.sensorRange.getDistance(DistanceUnit.INCH));
    }//loop end


    // Code to Run When Coach Hits STOP
    @Override
    public void stop()
    {
        telemetry.addData("Robot", "Stopped");
    }

    public void liftDrive(double speed, double LMotorInches, double timeoutS) {                                                                                                                                                                                                                 // :)

        int LMotorTarget;


        robot.LiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Determine new target position, and pass to motor controller
        LMotorTarget  = (int)(-LMotorInches * COUNTS_PER_INCH);

        //FLeftTarget  = robot.FmotorLeft.getCurrentPosition() + (int)(FLeftInches * COUNTS_PER_INCH);
        //FRightTarget = robot.FmotorRight.getCurrentPosition() + (int)(FRightInches * COUNTS_PER_INCH);
        //BLeftTarget = robot.BmotorLeft.getCurrentPosition() + (int)(BLeftInches * COUNTS_PER_INCH);
        //BRightTarget = robot.BmotorRight.getCurrentPosition() + (int)(BRightInches * COUNTS_PER_INCH);

        robot.LiftMotor.setTargetPosition(LMotorTarget);

        // Turn On RUN_TO_POSITION
        robot.LiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // reset the timeout time and start motion.
        //runtime.reset();
        robot.LiftMotor.setPower(Math.abs(speed));


            /*if (FLeftInches <0 && FRightInches <0) { //Backwards
                robot.BmotorLeft.setPower(-speed);
                robot.BmotorRight.setPower(-speed);
            }else if (FLeftInches <0 && FRightInches >0) { //Left
                robot.BmotorLeft.setPower(-speed);
                robot.BmotorRight.setPower(speed);
            }else if (FLeftInches >0 && FRightInches <0) { //Right
                robot.BmotorLeft.setPower(speed);
                robot.BmotorRight.setPower(-speed);
            }else{                                        //Forwards
                robot.BmotorLeft.setPower(speed);
                robot.BmotorRight.setPower(speed);
            }*/

        // keep looping while we are still active, and there is time left, and both motors are running.
        // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
        // its target position, the motion will stop.  This is "safer" in the event that the robot will
        // always end the motion as soon as possible.
        // However, if you require that BOTH motors have finished their moves before the robot continues
        // onto the next step, use (isBusy() || isBusy()) in the loop test.
            /*while (opModeIsActive() &&
                   (runtime.seconds() < timeoutS) &&
                   (robot.FmotorLeft.isBusy() && robot.FmotorRight.isBusy() && robot.BmotorLeft.isBusy() && robot.BmotorRight.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Target: ", "to %7d :%7d :%7d :%7d", FLeftTarget,  FRightTarget, BLeftTarget, BRightTarget);
                telemetry.addData("Postion:", "at %7d :%7d :%7d :%7d", robot.FmotorLeft.getCurrentPosition(),
                                                             robot.FmotorRight.getCurrentPosition(),robot.BmotorLeft.getCurrentPosition(),
                                                             robot.BmotorRight.getCurrentPosition());
                telemetry.update();
                RobotLog.d("%7d,%7d,%7d,%7d,%7d,", FLeftTarget,robot.FmotorLeft.getCurrentPosition(),
                                                             robot.FmotorRight.getCurrentPosition(),robot.BmotorLeft.getCurrentPosition(),
                                                             robot.BmotorRight.getCurrentPosition());
            }*/

        // Stop all motion;
        robot.LiftMotor.setPower(0);


        // Turn off RUN_TO_POSITION
        robot.LiftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //sleep(250);   // optional pause after each move
    }
    /*
     * This method scales the joystick input so for low joystick values, the
     * scaled value is less than linear.  This is to make it easier to drive
     * the robot more precisely at slower speeds.
     */
    double scaleInput(double dVal)  {
        double[] scaleArray = { 0.0, 0.05, 0.09, 0.10, 0.12, 0.15, 0.18, 0.20,
                //0.30, 0.40, 0.50, 0.55, 0.60, 0.65, 0.7, 0.75, 0.75};
                0.30, 0.36, 0.43, 0.50, 0.60, 0.72, 0.72, 0.85, 1.00 };
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