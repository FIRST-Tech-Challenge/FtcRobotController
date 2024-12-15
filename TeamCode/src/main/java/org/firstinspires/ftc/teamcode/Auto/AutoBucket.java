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
import org.firstinspires.ftc.teamcode.Auto.HardwareClassesNActions.OuttakeServos;
import org.firstinspires.ftc.teamcode.Auto.HardwareClassesNActions.SlideMotors;
import org.firstinspires.ftc.teamcode.Auto.HardwareClassesNActions.SlideServos;
import org.firstinspires.ftc.teamcode.MecanumDrive;

@Config
@Autonomous(name = "bucket", group = "Autonomous")
public class AutoBucket extends LinearOpMode {

    private Servos intakeThings;
    private OuttakeServos outtakeServos;
    private SlideMotors slideMotor;
    private SlideServos slideServos;

    @Override public void runOpMode(){
        //all of these are during init

        // instantiate your MecanumDrive at a particular pose.
        Pose2d initialPose = Positions.bucketInitialPos;
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        // make instance
        intakeThings = new Servos(hardwareMap);
        outtakeServos = new OuttakeServos(hardwareMap);
        slideMotor = new SlideMotors(hardwareMap);
        slideServos = new SlideServos(hardwareMap);


        //test path
        TrajectoryActionBuilder initToBlock1 = drive.actionBuilder(initialPose)
                .lineToY(drive.pose.position.y-3)
                .splineToConstantHeading(new Vector2d(47,40),Math.toRadians(280));

        TrajectoryActionBuilder block1Tobucket = drive.actionBuilder(initialPose)
                .waitSeconds(3)
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(53,58,Math.toRadians(45)),Math.toRadians(70))
                .waitSeconds(1)
                //Slides up
                //open claw
                .setTangent(Math.toRadians(225))
                .lineToX(drive.pose.position.x-3)
                //slides down
                .splineToLinearHeading(new Pose2d(58,40,Math.toRadians(270)),Math.toRadians(270))
                .waitSeconds(3)
                //pickup
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(53,58,Math.toRadians(45)),Math.toRadians(70))
                .waitSeconds(1)
                //Slides up
                //open claw
                .setTangent(Math.toRadians(225));


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
                        initToBlock1.build(),
                        intakeThings.openWait(),
                        intakeThings.clawGrab(),
                        outtakeServos.OuttakeClose(),
                        intakeThings.getOuttaTheWay(),
                        block1Tobucket.build()
//                        slideMotor.lifeMax(),
//                        outtakeServos.outtakeUp(),
//                        outtakeServos.outtakeOpen(),
//                        outtakeServos.OuttakeClose(),


                )
        );
    }
}
