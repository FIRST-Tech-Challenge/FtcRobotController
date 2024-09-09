/* Copyright (c) 2024 FIRST. All rights reserved.
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

package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

/**
 * This OpMode demonstrates the basics of using multiple vision portals simultaneously
 */
@TeleOp(name = "Concept: AprilTagMultiPortal", group = "Concept")
@Disabled
public class ConceptAprilTagMultiPortal extends LinearOpMode
{
    VisionPortal portal1;
    VisionPortal portal2;

    AprilTagProcessor aprilTagProcessor1;
    AprilTagProcessor aprilTagProcessor2;

    @Override
    public void runOpMode() throws InterruptedException
    {
        // Because we want to show two camera feeds simultaneously, we need to inform
        // the SDK that we want it to split the camera monitor area into two smaller
        // areas for us. It will then give us View IDs which we can pass to the individual
        // vision portals to allow them to properly hook into the UI in tandem.
        int[] viewIds = VisionPortal.makeMultiPortalView(2, VisionPortal.MultiPortalLayout.VERTICAL);

        // We extract the two view IDs from the array to make our lives a little easier later.
        // NB: the array is 2 long because we asked for 2 portals up above.
        int portal1ViewId = viewIds[0];
        int portal2ViewId = viewIds[1];

        // If we want to run AprilTag detection on two portals simultaneously,
        // we need to create two distinct instances of the AprilTag processor,
        // one for each portal. If you want to see more detail about different
        // options that you have when creating these processors, go check out
        // the ConceptAprilTag OpMode.
        aprilTagProcessor1 = AprilTagProcessor.easyCreateWithDefaults();
        aprilTagProcessor2 = AprilTagProcessor.easyCreateWithDefaults();

        // Now we build both portals. The CRITICAL thing to notice here is the call to
        // setLiveViewContainerId(), where we pass in the IDs we received earlier from
        // makeMultiPortalView().
        portal1 = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setLiveViewContainerId(portal1ViewId)
                .addProcessor(aprilTagProcessor1)
                .build();
        portal2 = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 2"))
                .setLiveViewContainerId(portal2ViewId)
                .addProcessor(aprilTagProcessor2)
                .build();

        waitForStart();

        // Main Loop
        while (opModeIsActive())
        {
            // Just show some basic telemetry to demonstrate both processors are working in parallel
            // on their respective cameras. If you want to see more detail about the information you
            // can get back from the processor, you should look at ConceptAprilTag.
            telemetry.addData("Number of tags in Camera 1", aprilTagProcessor1.getDetections().size());
            telemetry.addData("Number of tags in Camera 2", aprilTagProcessor2.getDetections().size());
            telemetry.update();
            sleep(20);
        }
    }
}
