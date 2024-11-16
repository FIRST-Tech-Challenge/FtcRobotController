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


@Autonomous(name="Unit_Test", group="Linear Opmode2")
public class Unit_Test extends CommonUtil {

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
        setZeroPowerBehavior();
        clawClosed();
        wristFlat();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        while (opModeIsActive()) {
//            bl.setPower(0.65);
//            fl.setPower(-0.6);
//            fr.setPower(0.65);
//            br.setPower(-0.55);
//
//            sleep(3000);
//            bl.setPower(0);
//            fl.setPower(0);
//            fr.setPower(0);
//            br.setPower(0);

//            moveForward_wDistance_wGyro(10,0.5);
//            sleep(500);
//
//            moveBackwards_wDistance_wGyro(10,0.5);
//            sleep(500);
//
//            moveSideways_wCorrection("right",10,0.35);
//            sleep(500);
//
//            moveSideways_wCorrection("left",10,0.35);
//            sleep(500);
//
//            turn("left",90);
//            sleep(500);
//
//            turn("right",90);
//            sleep(500);
//
//            intake(1000);
//            sleep(500);
//
//            extend(1,2000);
//            sleep(500);
//            clawClosed();
//            wristBent();
//            sleep(500);
//            clawOpen();
//            sleep(500);
//            clawClosed();
//            wristFlat();
//            sleep(500);
//            retract(1,2000);
//            sleep(500);
//            drone_Test();
//         encoder_test(200,0.3);

            armDown();
            sleep(5000);
            armUp();
            sleep(5000);
            armIdle();

         sleep(9000000);

        }
    }


}




