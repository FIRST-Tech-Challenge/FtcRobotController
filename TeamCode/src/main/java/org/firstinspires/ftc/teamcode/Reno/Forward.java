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

package org.firstinspires.ftc.teamcode.Reno;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This file illustrates the concept of driving a path based on time.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 *
 * The code assumes that you do NOT have encoders on the wheels,
 *   otherwise you would use: PushbotAutoDriveByEncoder;
 *
 *   The desired path in this example is:
 *   - Drive forward for 3 seconds
 *   - Spin right for 1.3 seconds
 *   - Drive Backwards for 1 Second
 *   - Stop and close the claw.
 *
 *  The code is written in a simple form with no optimizations.
 *  However, there are several ways that this type of sequence could be streamlined,
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name=" Nept-Forward and Turning ", group="Neptune")
public class Forward extends LinearOpMode {


    HardwareRobot robot   = new HardwareRobot();   // Use a Pushbot's hardware
    private ElapsedTime     runtime = new ElapsedTime();

    double power = 0.2;


    @Override
    public void runOpMode() {


        robot.init(hardwareMap);


        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();

        robot.leftDriveFront.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.leftDriveBack.setDirection(DcMotorSimple.Direction.REVERSE);
        waitForStart();

        runtime.reset();


        while (opModeIsActive() && (runtime.seconds() < 30.0))
        {
            // Moving forward
            telemetry.addData("Status", "ready to move forward for 4 seconds");
            power = 0.2;
            robot.leftDriveFront.setPower(power);
            robot.leftDriveBack.setPower(power);
            robot.rightDriveBack.setPower(power);
            robot.rightDriveFront.setPower(power);

            sleep(4000);

          this.robotStop();

            telemetry.addData("Status", "ready to move backward for 2 seconds");
            //Moving backward
            power = -0.2;

            robot.leftDriveFront.setPower(power);
            robot.leftDriveBack.setPower(power);
            robot.rightDriveBack.setPower(power);
            robot.rightDriveFront.setPower(power);

            sleep(2000);

           this.robotStop();


            telemetry.addData("Status", "ready to turn left for 1 second");
            //turning left
            power = 0.10;
            robot.leftDriveFront.setPower(-power);
            robot.leftDriveBack.setPower(-power);
            robot.rightDriveBack.setPower(power);
            robot.rightDriveFront.setPower(power);

            sleep(2000);


            this.robotStop();
            telemetry.addData("Status", "ready to move forward for 2 second");
            //Move Forward
            power = 0.2;
            robot.leftDriveFront.setPower(power);
            robot.leftDriveBack.setPower(power);
            robot.rightDriveBack.setPower(power);
            robot.rightDriveFront.setPower(power);

            sleep(2000);

            this.robotStop();
            telemetry.addData("Status", "ready to turn right for 1 seconds");
            //turning right
            power = 0.10;
            robot.leftDriveFront.setPower(power);
            robot.leftDriveBack.setPower(power);
            robot.rightDriveBack.setPower(-power);
            robot.rightDriveFront.setPower(-power);

            sleep(2000);

            this.robotStop();



            telemetry.update();
        }

        this.robotStop();
        telemetry.addData("Status" , "Program Finished");
        telemetry.update();


    }
    public void robotStop(){
      double  power = 0.0;

        robot.leftDriveFront.setPower(power);
        robot.leftDriveBack.setPower(power);
        robot.rightDriveBack.setPower(power);
        robot.rightDriveFront.setPower(power);

        sleep(1000);
    }
}
