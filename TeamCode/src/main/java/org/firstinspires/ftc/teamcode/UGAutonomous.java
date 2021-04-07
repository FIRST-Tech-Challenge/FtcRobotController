/* Copyright (c) 2019 FIRST. All rights reserved.
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
import static org.firstinspires.ftc.teamcode.MecanumDrive.MoveDirection.*;
import static org.firstinspires.ftc.teamcode.UGObjectDetector.ringStackState.*;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="Ultimate Goal Auto", group ="Concept")

public class UGAutonomous extends LinearOpMode {


    private MecanumDrive mecanumDrive = new MecanumDrive();
    private UGRobot robot = new UGRobot();
    private UGObjectDetector OD = new UGObjectDetector();




    @Override public void runOpMode() {

        mecanumDrive.init(hardwareMap, telemetry, this);
        robot.init(hardwareMap, telemetry, this);
        OD.init(hardwareMap, telemetry,this);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");

        while (!isStarted()) {
            UGObjectDetector.ringStackState ringsFound = OD.findRings();
            if (ringsFound == NONE) {
                telemetry.addLine("NONE Detected");
            } else if (ringsFound == SINGLE) {
                telemetry.addLine("SINGLE Detected");

            } else if (ringsFound == QUAD) {
                telemetry.addLine("QUAD Detected");

            } else {
                telemetry.addLine("This Cant Happen");
            }
            telemetry.update();
            this.idle();
        }

        waitForStart();

        if (opModeIsActive()) {
            UGObjectDetector.ringStackState ringsFound = OD.findRings();
            OD.shutdown();


            mecanumDrive.forward(60, 0.7);
            mecanumDrive.rightTurn(10, 0.2);
            robot.shoot(true);
            robot.shoot(true);
            robot.shoot(true);
            if (ringsFound == UGObjectDetector.ringStackState.NONE) {
                mecanumDrive.arcMove(0, 72, 0.3, RIGHT, true, true);
                mecanumDrive.forward(31, 0.5);
                mecanumDrive.backward(6, 0.5);
            } else if (ringsFound == UGObjectDetector.ringStackState.SINGLE) {
                mecanumDrive.rightTurn(2, 0.4);
                mecanumDrive.forward(37, 0.5);
                mecanumDrive.backward(19, 0.5);
            } else if (ringsFound == UGObjectDetector.ringStackState.QUAD) {
                mecanumDrive.rightTurn(16, 0.4);
                mecanumDrive.forward(72, 0.7);
                mecanumDrive.backward(55, 0.7);
            }
        } else {
            OD.shutdown();
        }

/*
        mecanumDrive.forward(24,.5);
                mecanumDrive.leftTurn(120, .5);
                robot.setPickup(UGRobot.pickupDirection.IN);
                mecanumDrive.backward(24, .5);
                robot.setPickup(UGRobot.pickupDirection.STOP);
                mecanumDrive.forward(24, .5);
                mecanumDrive.rightTurn(30, .5);
                mecanumDrive.forward(52, .5);
                mecanumDrive.rightTurn(180, .5);
                robot.setPickup(UGRobot.pickupDirection.OUT);
                sleep(500);
                robot.setPickup(UGRobot.pickupDirection.STOP);
                mecanumDrive.forward(12, .5);

        sleep(1500);

 */
    }
}
