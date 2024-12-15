/* Copyright (c) 2023 FIRST. All rights reserved.
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

package org.firstinspires.ftc.teamcode.Auto;

// RR-specific imports

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Auto.HardwareClassesNActions.Servos;
import org.firstinspires.ftc.teamcode.Auto.HardwareClassesNActions.SlideMotors;
import org.firstinspires.ftc.teamcode.MecanumDrive;

@Config
@Autonomous(name = "2 clip no servo", group = "Autonomous")
public class Clipsnoservo extends LinearOpMode {


    @Override public void runOpMode(){
        //all of these are during init

        // instantiate your MecanumDrive at a particular pose.
        Pose2d initialPose = Positions.clipsInitialPos;
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        //test path
        TrajectoryActionBuilder initToCLips = drive.actionBuilder(initialPose)
                .splineToConstantHeading(new Vector2d(-7,32),Math.toRadians(270))
                .waitSeconds(5)
                //slides down
                //Open claw
                .lineToX(-7)
                .lineToY(34)
                .setTangent(Math.toRadians(90))
                .waitSeconds(.5)
                //slides all the way down
                .splineToConstantHeading(new Vector2d(-27,38),Math.toRadians(180))
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(-46,13,Math.toRadians(90)),Math.toRadians(170))
                //foward
                .setTangent(Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(-46,59),Math.toRadians(90))
                .setTangent(Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(-46,48),Math.toRadians(270))
                .setTangent(Math.toRadians(90))
                .waitSeconds(3)
                .splineToConstantHeading(new Vector2d(-46,65),Math.toRadians(90))
                .waitSeconds(5)
                //slides up
                //open claw
                //close claw

                //Slides Up
                .setTangent(Math.toRadians(270))
                .splineToLinearHeading(new Pose2d(-7,32,Math.toRadians(270)),Math.toRadians(270));
        //slides down
        //Open claw

        // vision here that outputs position
        int visionOutputPosition = 1;

        // actions that need to happen on init; for instance, a claw tightening.
        //Actions.runBlocking(claw.closeClaw());
        while (!isStopRequested() && !opModeIsActive()) {
            int position = visionOutputPosition;
            telemetry.addData("Position during Init", position);
            telemetry.update();
        }
        int startPosition = visionOutputPosition;
        telemetry.addData("Starting Position", startPosition);
        telemetry.update();
        waitForStart();

        if (isStopRequested()) return;
        Actions.runBlocking(
                new SequentialAction(
                        initToCLips.build()
                )
        );
    }
}
