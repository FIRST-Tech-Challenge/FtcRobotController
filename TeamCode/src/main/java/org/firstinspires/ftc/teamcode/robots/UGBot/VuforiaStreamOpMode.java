package org.firstinspires.ftc.teamcode.robots.UGBot;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.teamcode.RC;

/*
 * This sample demonstrates how to stream frames from Vuforia to the dashboard. Make sure to fill in
 * your Vuforia key below and select the 'Camera' preset on top right of the dashboard. This sample
 * also works for UVCs with slight adjustments.
 */
@Autonomous
public class VuforiaStreamOpMode extends LinearOpMode {

    // TODO: fill in
    public static final String VUFORIA_LICENSE_KEY = "AbVe8An/////AAABmZuL3zqsJUfStpV5IU4Dp/p9KdvUSgvz7JuXGXwrFA4YEeDyH5BU3fbsp1mUKYLhA1WPX5r5E2nqv3sSkiP48oSuQRwWf7RTq7AfwxCY7qvldTj0ilT/XPb46/zyjbdZ7x/cQknV6zxt+rGLOiwRXID4wY/Tey52VMMoq1oxCFwogAXIWxZeF6DjmmfENbY6BwsXrAsIEHY3BQsdzI3HanDT6XJ+LUoPREvzi9Vh2iRhWiMX0E0pyWfs/El8qGl9tsQIEjaXp2Nax9zCKP8ehvr+8bwIF38qx+Rcmo1c9DH60fGFFzd4HW73UINTXwZvoJwCyh6KvBriLfDP8hcBXvStnd0JMi633BWsX5uZ+UiR";

    @Override
    public void runOpMode() throws InterruptedException {
        // gives Vuforia more time to exit before the watchdog notices
        msStuckDetectStop = 2500;

        VuforiaLocalizer.Parameters vuforiaParams = new VuforiaLocalizer.Parameters();
        vuforiaParams.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");
        vuforiaParams.vuforiaLicenseKey = RC.VUFORIA_LICENSE_KEY;
        vuforiaParams.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        VuforiaLocalizer vuforia = ClassFactory.getInstance().createVuforia(vuforiaParams);

        FtcDashboard.getInstance().startCameraStream(vuforia, 0);

        waitForStart();

        while (opModeIsActive());
    }
}