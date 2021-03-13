/**
 * Vuforia Test. Inits Vuforia and it
 * displays the target it sees, the location
 * of the phone/robot relative to the target,
 * and the angle/rotation of the phone.
 *
 * @author  Aamod
 * @version 1.0
 * @since   2020-July-10
 * @status: Fully working
 */

package org.firstinspires.ftc.teamcode.Autonomous.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Components.BasicChassis;
import org.firstinspires.ftc.teamcode.Components.Navigations.VuforiaWebcam;
import org.firstinspires.ftc.teamcode.Robot;

@Autonomous(name="VuforiaTest ", group="Tests: ")
//@Disabled
public class VuforiaTest extends LinearOpMode {


    @Override
    public void runOpMode() {
        telemetry.addData("beforeinit", 0);
        telemetry.update();
        VuforiaWebcam vuforiaWebcam = new VuforiaWebcam(this);
        telemetry.addData("afterinit", 0);
        telemetry.update();
        vuforiaWebcam.start();

        waitForStart();

        while (opModeIsActive()) {
            if(Math.sqrt(Math.pow(VuforiaWebcam.getVuforiaX(), 2) + Math.pow(VuforiaWebcam.getVuforiaY(), 2))>=24.5 && VuforiaWebcam.isTargetVisible()==true){
                telemetry.addData("ok dist", Math.hypot(VuforiaWebcam.getVuforiaX(),VuforiaWebcam.getVuforiaY()));
                telemetry.update();
            }

            if(Math.sqrt(Math.pow(VuforiaWebcam.getVuforiaX(), 2) + Math.pow(VuforiaWebcam.getVuforiaY(), 2))<24.5 && VuforiaWebcam.isTargetVisible()==true){
                telemetry.addData("Too far", Math.hypot(VuforiaWebcam.getVuforiaX(),VuforiaWebcam.getVuforiaY()));
                telemetry.update();
            }

        }

    }
}