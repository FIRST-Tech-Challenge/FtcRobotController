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

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Arcade", group="Arcade")
//@Disabled
public class ArcadeTeleOp extends LinearOpMode {

    DrivetrainHardware mDrive = new DrivetrainHardware();

    static int currentState;
    static final int IDLE = 0;
    static final int MANEUVERING = 1;
    static final int SHOOTING = 2;
    static final int COLLECTING = 3;

    @Override
    public void runOpMode()
    {

        mDrive.init(hardwareMap);


        Vision vision = new Vision(this);
        currentState = IDLE;
        waitForStart();

        while (opModeIsActive())
        {
            currentState = IDLE;
            drive(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);
            //runIntake();
            runFlyWheel();
            runPivot();
            runServos();
            doArm();
            //doVarious();
            runVoltageLED();
            //runRGBPatternSwitch();
        }
    }


    public void drive(double x, double y, double t)
    {

        double magnitude = Math.sqrt(x * x + y * y);
        double distance = Math.atan2(y, x);
        double turn = t/2;


        //calculate power with angle and magnitude

        double backLeft = magnitude * Math.sin(distance - Math.PI / 4) + turn;
        double backRight = magnitude * Math.sin(distance + Math.PI / 4) - turn;
        double frontLeft = magnitude * Math.sin(distance + Math.PI / 4) + turn;
        double frontRight = magnitude * Math.sin(distance - Math.PI / 4) - turn;

        /*in case the power to the motors gets over 1(as 1 is the maximum motor value, and in order
        to strafe diagonally, wheels have to move at different speeds), we divide them all by the
        highest value. This keeps them under 1, but in respect with each other*/

        if (magnitude != 0) {
            double divisor = 0;
            divisor = Math.max(Math.abs(backLeft), Math.abs(backRight));
            divisor = Math.max(divisor, Math.abs(frontLeft));
            divisor = Math.max(divisor, Math.abs(frontRight));

            telemetry.addData("divisor: ", divisor);

            backLeft = magnitude * (backLeft / divisor);
            backRight = magnitude * (backRight / divisor);
            frontLeft = magnitude * (frontLeft / divisor);
            frontRight = magnitude * (frontRight / divisor);
        }
        /*
        telemetry.addData("Magnitude: ", magnitude);
        telemetry.addData("turn: ", turn);
        telemetry.addData("backLeft: ", backLeft);
        telemetry.addData("backRight: ", backRight);
        telemetry.addData("frontLeft: ", frontLeft);
        telemetry.addData("frontRight: ", frontRight);
        telemetry.update();
        */

        mDrive.BL.setPower(backRight);
        mDrive.BR.setPower(backLeft);
        mDrive.FL.setPower(frontRight);
        mDrive.FR.setPower(frontLeft);

        telemetry.addData("FL", mDrive.FL.getCurrentPosition());
        telemetry.addData("FR", mDrive.FR.getCurrentPosition());
        telemetry.addData("BL", mDrive.BL.getCurrentPosition());
        telemetry.addData("BR", mDrive.BR.getCurrentPosition());
        telemetry.update();

        currentState = MANEUVERING;
    }

    public void runFlyWheel()
    {
        /*double shootingVel = gamepad2.left_trigger * 2500;
        mDrive.FlyWheel1.setVelocity(shootingVel);
        mDrive.FlyWheel2.setVelocity(shootingVel);*/

        mDrive.FlyWheel1.setPower(gamepad2.left_trigger);
        mDrive.FlyWheel2.setPower(gamepad2.left_trigger);

        if(gamepad2.left_trigger >= 0.2)
            currentState = SHOOTING;
    }

    /*public void runIntake() {
        if (gamepad1.left_trigger > 0.2) {
            mDrive.intake1.setPower(1);
            mDrive.intake2.setPower(-1);
            currentState = COLLECTING;
        } else if (gamepad1.right_trigger > 0.2) {
            mDrive.intake2.setPower(1);
            mDrive.intake1.setPower(-1);
            currentState = COLLECTING;
        } else {
            mDrive.intake1.setPower(0);
            mDrive.intake2.setPower(0);
        }
    }*/

    public void doArm()
    {
         if (gamepad2.right_stick_button)
             mDrive.Arm.setPower(gamepad2.right_stick_y);
         else
             mDrive.Arm.setPower(gamepad2.right_stick_y / 2);
    }

    public void runPivot()

    {
        mDrive.Intake.setPower(3 * gamepad2.left_stick_y / 4);
    }

    public void runServos()
    {
        if (gamepad2.dpad_up)
            mDrive.ringHopper.setPosition(1);
        else if (gamepad2.dpad_down)
            mDrive.ringHopper.setPosition(0);
        else
            mDrive.ringHopper.setPosition(0.5);

        if(gamepad2.x)
            mDrive.claw.setPosition(1);
        else if (gamepad2.y)
            mDrive.claw.setPosition(0);
    }
    public void doVarious()
    {
        /*
        telemetry.addData("Front left distance", mDrive.distanceFrontLeft.getDistance(DistanceUnit.CM));
        telemetry.addData("Front right distance", mDrive.distanceFrontRight.getDistance(DistanceUnit.CM));
        telemetry.addData("Back left distance", mDrive.distanceBackLeft.getDistance(DistanceUnit.CM));
        telemetry.addData("Back right distance", mDrive.distanceBackRight.getDistance(DistanceUnit.CM));
        */

        if(gamepad1.x)
        {
            telemetry.speak("still haven't asked");
            telemetry.update();
        }
        if(gamepad1.y)
        {
            telemetry.speak("At max rendering distance and I still can't find who asked.");
            telemetry.update();
        }
        if(gamepad1.a)
        {
            telemetry.speak("when, when, when did I ask LOLOLOLOLOLOLOLOL");
            telemetry.update();
        }
        if(gamepad1.b)
        {
            telemetry.speak("N G L T B H asked I M O");
            telemetry.update();
        }

    }

    public void runRGBPatternSwitch()
    {
        /*telemetry.addData("state: ", currentState);
        telemetry.update();
        switch (currentState)
        {
            case IDLE:
                mDrive.blink.setPattern(RevBlinkinLedDriver.BlinkinPattern.SHOT_WHITE);
            case MANEUVERING:
                mDrive.blink.setPattern(RevBlinkinLedDriver.BlinkinPattern.ORANGE);
            case COLLECTING:
                mDrive.blink.setPattern(RevBlinkinLedDriver.BlinkinPattern.STROBE_GOLD);
            case SHOOTING:
                mDrive.blink.setPattern(RevBlinkinLedDriver.BlinkinPattern.STROBE_RED);
            default:
                mDrive.blink.setPattern(RevBlinkinLedDriver.BlinkinPattern.ORANGE);
        }*/

    }

    public void runVoltageLED()
    {
        double voltage = mDrive.getVoltage();

        telemetry.addData("Voltage: ", voltage);
        telemetry.update();

        if (voltage > 13)
            mDrive.blink.setPattern(RevBlinkinLedDriver.BlinkinPattern.WHITE);
        else if (voltage > 12)
            mDrive.blink.setPattern(RevBlinkinLedDriver.BlinkinPattern.GOLD);
        else if (voltage > 11)
            mDrive.blink.setPattern(RevBlinkinLedDriver.BlinkinPattern.ORANGE);
        else
            mDrive.blink.setPattern(RevBlinkinLedDriver.BlinkinPattern.STROBE_RED);
    }
}
