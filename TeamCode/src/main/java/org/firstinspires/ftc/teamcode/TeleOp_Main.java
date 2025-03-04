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

// import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
// import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

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

@TeleOp(name="TeleOp_Main", group="Iterative_TeleOp")

public class TeleOp_Main extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    // Declare hardware variables
        // Declare drive motors
        private DcMotor driveFL = null;
        private DcMotor driveFR = null;
        private DcMotor driveBL = null;
        private DcMotor driveBR = null;

        // Declare slide motors
        private DcMotor slideL = null;
        private DcMotor slideR = null;

        // Declare winch motors
        private DcMotor winch1 = null;
        private DcMotor winch2 = null;

    // Declare code data variables
        double Y  = 0;
        double X  = 0;
        double rX = 0;
        double d = 0;

        int speedPercent = 65;
        
        float SlidePow = 0;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        // ---------------------------------------------------------------------------------------
        // Initialize drive motors
            driveFL  = hardwareMap.get(DcMotor.class, "FL");
            driveFR  = hardwareMap.get(DcMotor.class, "FR");
            driveBL  = hardwareMap.get(DcMotor.class, "BL");
            driveBR  = hardwareMap.get(DcMotor.class, "BR");
        // Initialize slide motors
            slideL   = hardwareMap.get(DcMotor.class, "Lslide");
            slideR   = hardwareMap.get(DcMotor.class, "Rslide");
        // Initialize winch motors
            winch1   = hardwareMap.get(DcMotor.class, "Winch1");
            winch2   = hardwareMap.get(DcMotor.class, "Winch2");

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        driveFL.setDirection(DcMotor.Direction.REVERSE);
        driveFR.setDirection(DcMotor.Direction.FORWARD);
        driveBL.setDirection(DcMotor.Direction.REVERSE);
        driveBR.setDirection(DcMotor.Direction.FORWARD);

        // Define the directions of the slide and winch motors
        slideL.setDirection(DcMotor.Direction.FORWARD);
        slideR.setDirection(DcMotor.Direction.FORWARD);
        winch1.setDirection(DcMotor.Direction.FORWARD);
        winch2.setDirection(DcMotor.Direction.FORWARD);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit START
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits START
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits START but before they hit STOP
     */
    @Override
    public void loop() {

        UpdateDrivetrain();
        UpdateSlides();
        
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

    /**
     * This function handles the math for the mecanum drivetrain
     */
    private void UpdateDrivetrain() {

        UpdateSpeedLimiter();

        Y = -gamepad1.left_stick_y;
        X = (gamepad1.left_stick_x * 1.1);
        rX = gamepad1.right_stick_x;
        d = JavaUtil.maxOfList(JavaUtil.createListWith(JavaUtil.sumOfList(JavaUtil.createListWith(Math.abs(Y), Math.abs(X), Math.abs(rX))), 1));

        driveFL.setPower(((Y + X + rX) / d) * ((double) speedPercent / 100));
        driveBL.setPower(((Y - X + rX) / d) * ((double) speedPercent / 100));
        driveFR.setPower(((Y - X - rX) / d) * ((double) speedPercent / 100));
        driveBR.setPower(((Y + X - rX) / d) * ((double) speedPercent / 100));
    }

    /**
     * This function handles the speed limiting for the drivetrain
     */
    private void UpdateSpeedLimiter() {
        if (gamepad1.right_trigger > 0.1) {
            speedPercent = 100;
        } else if (gamepad1.left_trigger > 0.1) {
            speedPercent = 33;
        } else {
            speedPercent = 66;
        }
        telemetry.addData("Speed Percentage", speedPercent);
    }

    /**
     * Describe this function...
     */
    private void UpdateSlides() {
        SlidePow = Math.abs(gamepad2.right_stick_y * 1);
        if (gamepad2.right_stick_button) {
            slideL.setTargetPosition(-1545);
            slideR.setTargetPosition(-1545);
            slideL.setPower(1);
            slideR.setPower(1);
            slideL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slideR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        } else {
            if (gamepad2.right_stick_y > 0.05) {
                slideL.setTargetPosition(0);
                slideR.setTargetPosition(0);
                slideL.setPower(SlidePow);
                slideR.setPower(SlidePow);
                slideL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slideR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            } else if (gamepad2.right_stick_y < -0.05) {
                slideL.setTargetPosition(-5000);
                slideR.setTargetPosition(-5000);
                slideL.setPower(SlidePow);
                slideR.setPower(SlidePow);
                slideL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slideR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            } else {
                slideR.setTargetPosition(slideR.getCurrentPosition());
                slideL.setTargetPosition(slideL.getCurrentPosition());
                slideR.setPower(1);
                slideL.setPower(1);
                slideR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slideL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            if (slideR.getCurrentPosition() > -25 && !(gamepad2.right_stick_y < -0.05)) {
                ((DcMotorEx) slideL).setMotorDisable();
                ((DcMotorEx) slideR).setMotorDisable();
            } else {
                ((DcMotorEx) slideL).setMotorEnable();
                ((DcMotorEx) slideR).setMotorEnable();
            }
        }
        telemetry.addData("Slide Power", SlidePow);
        telemetry.addData("Left Slide Power", slideL.getPower());
        telemetry.addData("Current Left Slide Position", slideL.getCurrentPosition());
        telemetry.addData("Current Left Slide Target", slideL.getTargetPosition());
        telemetry.addData("Left Slide Current", ((DcMotorEx) slideL).getCurrent(CurrentUnit.AMPS));
        telemetry.addData("Right Slide Power", slideR.getPower());
        telemetry.addData("Current Right Slide Position", slideR.getCurrentPosition());
        telemetry.addData("Current Right Slide Target", slideR.getTargetPosition());
        telemetry.addData("Right Slide Current", ((DcMotorEx) slideR).getCurrent(CurrentUnit.AMPS));
    }

}
