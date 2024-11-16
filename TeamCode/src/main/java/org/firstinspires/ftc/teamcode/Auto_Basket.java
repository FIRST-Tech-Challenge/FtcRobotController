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
//MINE ( AARUSH )
package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.

 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.

 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */


@Autonomous(name="Auto_Basket", group="Linear Opmode2")
public class Auto_Basket extends CommonUtil {

    Orientation myRobotOrientation;

    @Override
    public void runOpMode() {

        //setup
        telemetry.setAutoClear(false);
        // initialize hardware
        initialize(hardwareMap);
        // Initialize motors
        setMotorOrientation();
        //resetMotorEncoderCounts();
        setMotorToZeroPower();
        clawClosed();
        wristFlat();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        while (opModeIsActive()) {

            moveBackwards_wDistance_wGyro(24,0.5);
            sleep(200);

            slideUp(1,500);
            sleep(200);

            basketDown();
            sleep(1000);
            slideDown(0.8,500);
            sleep(200);

            moveForward_wDistance_wGyro(2,0.3);
            moveSideways_wCorrection("left",12,0.6);
            sleep(200);

            turn("left",90);
            sleep(200);

            armDown();
            intakeOn();

            moveBackwards_wDistance_wGyro(10,0.5);
            sleep(200);

            armUp();
            intakeReverse();
            sleep(300);
            intakeOff();

            armDown(); //idle position
            turn("right",90);
            sleep(200);

            moveSideways_wCorrection("left",2,0.6);
            sleep(200);

            slideUp(1,2000);
            sleep(200);

            moveBackwards_wDistance_wGyro(2,0.5);
            sleep(200);

            //then we place the sample and repeat process boi











            sleep(9000000);

        }
    }


}




