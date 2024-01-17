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
import com.arcrobotics.ftclib.command.Robot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robotContainer.Constants;
import org.firstinspires.ftc.teamcode.robotContainer.RobotContainer;

/*
TeleOp OpMode script using Command-based Robot
 */

@TeleOp(name="TeleOp_OpMode", group="TeleOp")
//@Disabled
public class TeleOp_OpMode extends LinearOpMode {

    @Override
    public void runOpMode() {

        //Robot initial pose
        Pose2d initialPose = PoseStorage.currentPose;
        double allianceHeadingOffset = PoseStorage.allianceHeadingOffset;

        //Instantiate the robot
        RobotContainer m_robot = new RobotContainer(
                Constants.OpModeType.TELEOP,
                hardwareMap,
                telemetry,
                gamepad1,
                gamepad2,
                initialPose,
                allianceHeadingOffset);

        //Wait for driver to press PLAY and then STOP
        waitForStart();

<<<<<<< Updated upstream
        //Disable the parking detection pipeline and start the parking timer countdown
        m_robot.disableVision();

        // Run the robot until the end of the match (or until the driver presses STOP)
        while (opModeIsActive() && !isStopRequested())
        {
            m_robot.run();
        }

        //Reset the PoseStorage in case we have to start over
=======
        //Switches from color pipeline to april tag detection
      //  m_robot.switchToApril();

        // Run the robot until the end of the match (or until the driver presses STOP)
        if(opModeIsActive()) {
            while (opModeIsActive() && !isStopRequested()) {
                m_robot.run();

            }
            m_robot.disableVision();
        }


        //Reset the PoseStorage in case we have to start over

>>>>>>> Stashed changes
        PoseStorage.currentPose = new Pose2d(new Vector2d( 0.0, 0.0), 0.0);
        m_robot.reset();
    }
}
