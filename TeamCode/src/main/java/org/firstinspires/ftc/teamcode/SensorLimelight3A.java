/*
Copyright (c) 2024 Limelight Vision

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of FIRST nor the names of its contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;
@TeleOp(name = "Lime Camera", group = "Sensor")
@Disabled
public class SensorLimelight3A extends LinearOpMode {

    // Neha: This is what was causing some of your problems on Sunday. Ask me to explain how static variables keep their
    // value outside of the program and none of our variables should ever be static.
    // Neha: Set all MIN values to 500 and all MAX values to -500
    // Neha: Min always comes before max
    private double X_MAX = -100;
    private double Y_MAX = -100;
    private double YAW_MAX = -500;
    private double X_MIN = 100;
    private double Y_MIN = 500;
    private double YAW_MIN = 100;
    private double X_CURRENT = 0;
    private double Y_CURRENT = 0;
    private double YAW_CURRENT = 0;

    private Limelight3A limelight;
    //The limelight has a heading range of 62 degrees and can see the AprilTag from 10 feet away. Can be positioned at max 14 1/2 inches high.
    @Override
    public void runOpMode() throws InterruptedException {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        telemetry.setMsTransmissionInterval(11);
        limelight.pipelineSwitch(0);
        // Starts polling for data. If you neglect to call start(), getLatestResult() will return null.
        limelight.start();
        telemetry.addData(">", "Robot Ready.  Press Play.");
        telemetry.update();
        waitForStart();
        while (opModeIsActive()) {
            LLStatus status = limelight.getStatus();
            telemetry.addData("Name", "%s", status.getName());
            telemetry.addData("LL", "Temp: %.1fC, CPU: %.1f%%, FPS: %d", status.getTemp(), status.getCpu(), (int) status.getFps());   //Tells us the temperatures, CPU and FPS
            telemetry.addData("Pipeline", "Index: %d, Type: %s", status.getPipelineIndex(), status.getPipelineType());              //Tells us pipeline value

            LLResult result = limelight.getLatestResult();
            if (result != null) {
                // Access general information
                double targetingLatency = result.getTargetingLatency();
                telemetry.addData("Total Latency", "%.0f", result.getTargetingLatency());

                if (result.isValid()) {
                    Pose3D botpose = result.getBotpose();
                    // Access AprilTag ID results
                    List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
                    for (LLResultTypes.FiducialResult fr : fiducialResults) {
                        telemetry.addData("AprilTag ID", "%d", fr.getFiducialId());             // AprilTag ID
                    }
                    telemetry.addData("X-Location", "%.1f", botpose.getPosition().x);            // Get X-Location from the robot
                    telemetry.addData("Y-Location", "%.1f", botpose.getPosition().y);            // Get Y-Location from the robot
                    telemetry.addData("Heading", "%.0f", botpose.getOrientation().getYaw());

                    X_CURRENT = botpose.getPosition().x;
                    Y_CURRENT = botpose.getPosition().y;
                    YAW_CURRENT = botpose.getOrientation().getYaw();

                    if (X_CURRENT > X_MAX) {
                        X_MAX = X_CURRENT;
                    }
                    if (X_CURRENT < X_MIN) {
                        X_MIN = X_CURRENT;
                    }
                    if (Y_CURRENT > Y_MAX) {
                        Y_MAX = Y_CURRENT;
                    }
                    if (Y_CURRENT < Y_MIN) {
                        Y_MIN = Y_CURRENT;
                    }
                    if (YAW_CURRENT > YAW_MAX) {
                        YAW_MAX = YAW_CURRENT;
                    }
                    if (YAW_CURRENT < YAW_MIN) {
                        YAW_MIN = YAW_CURRENT;
                    }
                }
                // Neha: Use 2 decimal points for all values.
                telemetry.addData("X Max", "%.1f", X_MAX);
                telemetry.addData("X Min", "%.1f", X_MIN);
                telemetry.addData("Y Max", "%.1f", Y_MAX);
                telemetry.addData("Y Min", "%.1f", Y_MIN);
                telemetry.addData("Yaw Max", "%.0f", YAW_MAX);
                telemetry.addData("Yaw Min", "%.0f", YAW_MIN);
                telemetry.update();
                RobotLog.vv("RockinRobots", "Yaw Current: %.0f, Yaw Max: %.0f", YAW_MIN, YAW_MAX);
            }
        }


        limelight.stop();
    }
}