package org.firstinspires.ftc.teamcode.opModes;
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

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.odometry.Odometry;
import org.firstinspires.ftc.teamcode.odometry.OdometryWheel;
import org.firstinspires.ftc.teamcode.odometry.PhysicalOdoWheel;
import org.firstinspires.ftc.teamcode.utility.pose;

import java.util.ArrayList;
import java.util.List;

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

@TeleOp(name="OdoTest", group="Iterative Opmode")

public class OdoTest extends OpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    Odometry odometry;

    DcMotor frontRight;
    DcMotor frontLeft;
    DcMotor backRight;
    DcMotor backLeft;

    PhysicalOdoWheel frontRightOdo;
    PhysicalOdoWheel frontLeftOdo;
    PhysicalOdoWheel backRightOdo;
    PhysicalOdoWheel backLeftOdo;
//    pose deltaPose;
    pose globalPosition;
    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initializing...");

        //physical wheels
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");

        //odometry wheels //todo put the correct offsets
        frontRightOdo = new PhysicalOdoWheel(new pose(178.5,168,Math.PI/2), frontRight);
        frontLeftOdo = new PhysicalOdoWheel(new pose(-178.5,168,Math.PI/2), frontLeft);
        backRightOdo = new PhysicalOdoWheel(new pose(178.5,-168,Math.PI/2), backRight);
        backLeftOdo = new PhysicalOdoWheel(new pose(-178.5,-168,Math.PI/2), backLeft);

        List<OdometryWheel> odometryWheels = new ArrayList<>();
        odometryWheels.add(frontLeftOdo);
        odometryWheels.add(frontRightOdo);
        odometryWheels.add(backLeftOdo);
        odometryWheels.add(backRightOdo);

        // odometry system
        pose initial = new pose(0,0,0);
        odometry = new Odometry(initial, odometryWheels);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {

    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        odometry.start();

        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {

        globalPosition = odometry.getPosition();

        // output current delta
//        telemetry.addData("Delta", "Delta X: " + deltaPose.x
//                + "Delta Y: " + deltaPose.y + "Delta R: " + deltaPose.r);

        // output current pose

        telemetry.addData("Position", globalPosition.toString());
        telemetry.addData("front Right.GetCurPos ", frontRightOdo.getWheelPosition());
        telemetry.addData("front Left.GetCurPos", frontLeftOdo.getWheelPosition());
        telemetry.addData("back Right.GetCurPos ", backRightOdo.getWheelPosition());
        telemetry.addData("back Left.GetCurPos", backLeftOdo.getWheelPosition());
        telemetry.addData("Detla front Right: ", frontRightOdo.getDeltaPosition());
        telemetry.addData("Detla front Left: ", frontLeftOdo.getDeltaPosition());
        telemetry.addData("Detla back Right: ", backRightOdo.getDeltaPosition());
        telemetry.addData("Detla back Left: ", backLeftOdo.getDeltaPosition());
//        telemetry.addData("Position", "Current X: " +  globalPosition.x);
//        telemetry.addData(  "Position","Current Y:" +  globalPosition.y );
//        telemetry.addData("Position", "Current R:" + globalPosition.r);

        telemetry.addData("Status", "Run Time: " + runtime.toString());
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        odometry.end();
    }

}



