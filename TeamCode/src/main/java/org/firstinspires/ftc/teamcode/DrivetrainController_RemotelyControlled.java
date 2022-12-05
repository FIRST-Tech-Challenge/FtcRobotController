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

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveKinematics;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Based off of BasicOpMode_Iterative
 */

@TeleOp(name="Holonomic Mechanum Remote Control", group="Driver Opmode")

public class DrivetrainController_RemotelyControlled extends OpMode
{
    // Declare OpMode members.
    private final ElapsedTime runtime = new ElapsedTime();
    private Motor frontLeftDrive = null;
    private Motor frontRightDrive = null;
    private Motor rearLeftDrive = null;
    private Motor rearRightDrive = null;
    private MecanumDrive drivetrain;
    private MecanumDriveKinematics kinematics;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        initDrivetrain();
        initOdometry();
        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");


    }

    private void initOdometry() {
        //CPR = Ratio*PPR*4
        int cpr = 20 * 7 * 4;
        double rpm = 273.7882;
        frontLeftDrive = new Motor(hardwareMap, "front_left_drive", cpr, rpm);
        frontRightDrive = new Motor(hardwareMap, "front_right_drive", cpr, rpm);
        rearLeftDrive = new Motor(hardwareMap, "rear_left_drive", cpr, rpm);
        rearRightDrive = new Motor(hardwareMap, "rear_right_drive", cpr, rpm);
        drivetrain = new MecanumDrive(frontLeftDrive, frontRightDrive, rearLeftDrive, rearRightDrive);

        Translation2d wheelFrontLeftLoc = new Translation2d(6,8.616);
        Translation2d wheelRearLeftLoc = new Translation2d(-6,8.616);
        Translation2d wheelFrontRightLoc = new Translation2d(6, -8.616);
        Translation2d wheelRearRightLoc = new Translation2d(-6,8.616);
        kinematics = new MecanumDriveKinematics(wheelFrontLeftLoc, wheelFrontRightLoc, wheelRearLeftLoc, wheelRearRightLoc);
    }

    private void initDrivetrain() {

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
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        drivetrain.driveRobotCentric(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);

        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
    }
    private double hypotenuse(double a,double b) {
        return Math.sqrt((a*a)+(b*b));
    }
    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
