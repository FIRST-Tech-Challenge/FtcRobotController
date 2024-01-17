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

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robotContainer.Constants;
import org.firstinspires.ftc.teamcode.robotContainer.RobotContainer;

/*
Autonomous OpMode script using Command-based Robot
 */

@Autonomous(name="RedLeft_Auton", group="Autonomous")
//@Disabled
public class RedLeft_Auton extends LinearOpMode {

    @Override
    public void runOpMode() {

        //Initialize the robot's Pose
//        Pose2d initialPose = new Pose2d(new Vector2d( 35.0, 60.0), -90.0);
//        Pose2d initialPose = new Pose2d(new Vector2d( 40.50, -65.0), Math.toRadians(90.0));
<<<<<<< Updated upstream
        Pose2d initialPose = new Pose2d(new Vector2d( 40, 65.0), Math.toRadians(-90.0));

        //Instantiate the robot
        RobotContainer m_robot = new RobotContainer(
                Constants.OpModeType.BLUE_LEFT_AUTO,
=======
        Pose2d initialPose = new Pose2d(new Vector2d( -32, -64.0), Math.toRadians(-270.0));

        //Instantiate the robot
        RobotContainer m_robot = new RobotContainer(
                Constants.OpModeType.RED_LEFT_AUTO,
>>>>>>> Stashed changes
                hardwareMap,
                telemetry,
                gamepad1,
                gamepad2,
                initialPose, 0.0);

        getRuntime();

        //Wait for driver to press PLAY
        waitForStart();

        //reset the runtime timer
//        resetRuntime();

        //Disable the parking detection pipeline and start the parking timer countdown
        m_robot.disableVision();

        // Run the robot until the end of the match (or until the driver presses STOP)
        while (opModeIsActive() && !isStopRequested())
        {
//            m_robot.setCurrentTime(getRuntime());
            m_robot.run();
        }

        //Store the last post of the robot to a static
        PoseStorage.currentPose = m_robot.getRobotPose();
        PoseStorage.allianceHeadingOffset = -90.0; //red side
        m_robot.reset();
    }
}
