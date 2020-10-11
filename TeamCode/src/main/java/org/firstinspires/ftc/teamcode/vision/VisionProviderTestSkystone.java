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

package org.firstinspires.ftc.teamcode.vision;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Demonstrates empty OpMode
 */
@TeleOp(name = "Skystone VisionProvider Test", group = "Linear Opmode")
public class VisionProviderTestSkystone extends LinearOpMode {

    private static final Class<? extends SkystoneVisionProvider>[] visionProviders = VisionProvidersSkystone.visionProviders;
    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Configuration");
        telemetry.update();
        int visionProviderState = 0;
        boolean toggle = false;
        boolean visionProviderFinalized = false;
        SkystoneVisionProvider vp = null;
        while (!isStarted()) {
            if (!visionProviderFinalized && gamepad1.dpad_left && !toggle) {
                toggle = true;
                visionProviderState++;
                if(visionProviderState == visionProviders.length)
                    visionProviderState = 0;
            } else if (!visionProviderFinalized && !gamepad1.dpad_left){
                toggle = false;
            }
            if (!visionProviderFinalized && gamepad1.dpad_up) {
                try {
                    telemetry.addData("Please wait","Initializing vision");
                    telemetry.update();
                    vp = visionProviders[visionProviderState].newInstance();
                    vp.initializeVision(hardwareMap, telemetry, true, Viewpoint.WEBCAM, true);
                } catch (IllegalAccessException | InstantiationException e) {
                    throw new RuntimeException(e);
                }
                visionProviderFinalized = true;
            }
            telemetry.addData("Status", "VisionBackend: %s (%s)", visionProviders[visionProviderState].getSimpleName(), visionProviderFinalized ? "finalized" : System.currentTimeMillis()/500%2==0?"**NOT FINALIZED**":"  NOT FINALIZED  ");
            telemetry.update();
        }
        telemetry.addData("Status", "Started");
        telemetry.update();
        if (vp == null) {
            try {
                telemetry.addData("Please wait","Initializing vision");
                telemetry.update();
                vp = visionProviders[visionProviderState].newInstance();
                vp.initializeVision(hardwareMap, telemetry, true, Viewpoint.WEBCAM, true);
            } catch (IllegalAccessException | InstantiationException e) {
                throw new RuntimeException(e);
            }
        }
        while (opModeIsActive()) {
            SkystoneTargetInfo target = vp.detectSkystone();
            telemetry.addData("VisionDetection", "%s", target);
            if(!target.quarryPosition.equals(StonePos.NONE_FOUND))
                vp.reset();
            telemetry.update();
        }
        vp.shutdownVision();
    }
}
