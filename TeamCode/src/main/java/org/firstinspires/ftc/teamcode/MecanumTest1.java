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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import static java.lang.Math.sqrt;


@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="Mecanum Test 1", group ="Tests")

public class MecanumTest1 extends LinearOpMode {


    private ElapsedTime runtime = new ElapsedTime();
    private HPMC[] motors = new HPMC[4];
    private HPMC rightManipulator = null;
    private HPMC leftManipulator = null;
    private DistanceSensor sensorRange;
    private double[] power = new double[4];
    private final double MOTOR_SPEED = 2800;
    boolean simpleMode = true;

    private final int FL = 0;
    private final int FR = 1;
    private final int BL = 2;
    private final int BR = 3;
    private final double SLOW = 0.6;

    private final double MSPEED = 1.0;
    private double updates = 0;
    MecanumDrive mecanumDrive = new MecanumDrive();

    @Override
    public void runOpMode() {

        telemetry.addData("Status", "Initialized");
        //test live UnsupportedOperationException
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).

        mecanumDrive.init(hardwareMap, telemetry, this);


        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");


        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        waitForStart();

        /*
        capstone.setPosition(1);
        sleep(1000);
        capstone.setPosition(0);
        sleep(1000);
        capstone.setPosition(1);
        */


        System.out.println("Starting forward 24");
        mecanumDrive.forward(24, 0.7);

        sleep(500);

        status("Starting backward 24");
        mecanumDrive.backward(24, 0.4);
        sleep(500);



        status("Starting right 90");

        mecanumDrive.rightTurn(90, 0.3);
        sleep(500);

        mecanumDrive.rightTurn(90, 0.2);
        sleep(500);

        mecanumDrive.rightTurn(90, 0.5);
        sleep(500);

        mecanumDrive.rightTurn(90, 0.4);
        sleep(500);

        mecanumDrive.leftTurn( 360, 0.8);
        //mecanumDrive.leftTurn(360, 0.5);

        sleep(1000);
        /*
        status("Starting right 90 2");

        mecanumDrive.rightTurn(90, 0.5);
        telemetry.addData("Status" , "Starting right 90");
        telemetry.update();
        sleep(3000);

        mecanumDrive.rightTurn(90, 0.5);
        status("Starting right 90");
        sleep(3000);

        mecanumDrive.rightTurn(90, 0.5);
        System.out.println("Done");
        */
        status( "Done");

    }
    void status(String string) {
        telemetry.addData("Status", string);
        telemetry.update();
    }
}

