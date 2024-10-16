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
package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ControlSystem;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.drivetrains.Mecanum;
import org.firstinspires.ftc.teamcode.roadrunner.Drawing;
import org.firstinspires.ftc.teamcode.subsystems.SparkOdo;
import org.firstinspires.ftc.teamcode.utils.*;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

/*
 * This OpMode illustrates how to use the Limelight3A Vision Sensor.
 *
 * @see <a href="https://limelightvision.io/">Limelight</a>
 *
 * Notes on configuration:
 *
 *   The device presents itself, when plugged into a USB port on a Control Hub as an ethernet
 *   interface.  A DHCP server running on the Limelight automatically assigns the Control Hub an
 *   ip address for the new ethernet interface.
 *
 *   Since the Limelight is plugged into a USB port, it will be listed on the top level configuration
 *   activity along with the Control Hub Portal and other USB devices such as webcams.  Typically
 *   serial numbers are displayed below the device's names.  In the case of the Limelight device, the
 *   Control Hub's assigned ip address for that ethernet interface is used as the "serial number".
 *
 *   Tapping the Limelight's name, transitions to a new screen where the user can rename the Limelight
 *   and specify the Limelight's ip address.  Users should take care not to confuse the ip address of
 *   the Limelight itself, which can be configured through the Limelight settings page via a web browser,
 *   and the ip address the Limelight device assigned the Control Hub and which is displayed in small text
 *   below the name of the Limelight on the top level configuration screen.
 */
@Config
@TeleOp(name = "LimelightTest")
public class LimelightTest extends LinearOpMode {
    private IMU imu;
    private LimeLightWrapper wrapper;
    private Limelight3A limelight3A;

    private Pose2d positionMT1, positionMT2;
//poopoo
    @Override
    public void runOpMode() throws InterruptedException
    {

        /*
        LIME LIGHT TAKE AWAY:
        ROTATION IS INVERTED (For spark rotation anyway)
         */





        positionMT1 = new Pose2d(0,0,0);
        positionMT2 = new Pose2d(0,0,0);
        Mecanum robot = new Mecanum(hardwareMap);

        GamepadEvents controller1 = new GamepadEvents(gamepad1);

        limelight3A = hardwareMap.get(Limelight3A.class,"limelight");
        wrapper = new LimeLightWrapper(limelight3A);
//        imu = hardwareMap.get(IMU.class, "imu");
//        imu.initialize(new IMU.Parameters(
//                new RevHubOrientationOnRobot(
//                        RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
//                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
//                )
//        ));
        SparkOdo sparkOdo = new SparkOdo(hardwareMap);



        waitForStart();


        while (opModeIsActive()) {
            robot.drive(controller1.left_stick_y, controller1.left_stick_x, controller1.right_stick_x);
            //For some reason, angle direction is inverted, this may be due to the offset.
            Pose3D pose3D = wrapper.distanceFromTag(-Math.toDegrees(sparkOdo.getPos().h));
            Pose3D pose3D_MT1 = wrapper.distanceFromTag();
            if(pose3D!=null) {
                Pose3D pos1 = wrapper.inToMM(pose3D_MT1);
                telemetry.addLine("Mega Tag 1:");
                telemetry.addData("1 - X: ",pos1.getPosition().x);
                telemetry.addData("1 - Y: ",pos1.getPosition().y);
                telemetry.addData("1 - Z: ",pos1.getPosition().z);
                telemetry.addData("Calculated Rotation: ",pos1.getOrientation().getYaw(AngleUnit.DEGREES));

                Pose3D pos = wrapper.inToMM(pose3D);
                telemetry.addLine("Mega Tag 2:");
                telemetry.addData("2 - X: ",pos.getPosition().x);
                telemetry.addData("2 - Y: ",pos.getPosition().y);
                telemetry.addData("2 - Z: ",pos.getPosition().z);

                positionMT1 = new Pose2d(pos1.getPosition().x,
                        pos1.getPosition().y,
                        pos1.getOrientation().getYaw(AngleUnit.RADIANS));

                positionMT2 = new Pose2d(pos.getPosition().x,
                        pos.getPosition().y,
                        -sparkOdo.getPos().h);

            }


            if(controller1.a.onPress()){
                sparkOdo.resetOdo();
            }


            drawDashboard();
            telemetry.addData("Rotation: ",Math.toDegrees(sparkOdo.getPos().h));
            telemetry.update();
            controller1.update();
        }

    }

    public void drawDashboard(){
        TelemetryPacket packet = new TelemetryPacket();
        Canvas canvas = packet.fieldOverlay();
//        canvas.drawImage("/dash/into-the-deep.png", 0, 0, 144, 144);
        canvas.setStroke("#3F51B5");
        Drawing.drawRobot(canvas, positionMT1);
        Drawing.drawRobot(canvas, positionMT2);
        FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }


}
