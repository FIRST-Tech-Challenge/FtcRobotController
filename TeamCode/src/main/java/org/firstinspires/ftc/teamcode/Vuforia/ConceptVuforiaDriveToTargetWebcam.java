package org.firstinspires.ftc.teamcode.Vuforia;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

@TeleOp(name="Vuforia Test", group = "Concept")
public class ConceptVuforiaDriveToTargetWebcam extends LinearOpMode
{
    final double MM_PER_INCH = 25.40 ;   //  Metric conversion

    private static final String VUFORIA_KEY =
            "AQXJtf3/////AAABmRNtqRHOi01ztX4zAd+KuPdOXWfS1wa0VbmgatYieybPLp6S7F+PpEIxktdGYJhBEd9SBRox+HmVviMD3XQdouhnWR8NtrjrlFXN63orJIBQHEhUC8vZTbO1Sq/EsAELA4VhNOHmERzTXhJ4Kz8h4Cy4tJr192IiMB7W02Czjts055dln8QTMQPXqXgFU4qeY7mxKrpFco1pKT/OFCdwVHWWvkozJBXEIujp/eqjknmpKgrlPQPRkFOzyLRHHCybs/R9Agw6EDiVGgE92XCU/lFOaU0U92R/WlWoFGh/LlyHgqlFg6N/SskENvsnX1j7tbCF68Bz78meWiaRYSktMJRgshC1Fk3G7EnmMKaCPFMJ";

    VuforiaLocalizer vuforia    = null;
    OpenGLMatrix targetPose     = null;

    @Override public void runOpMode()
    {
        // Set up camera
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        // Turn off Extended tracking.  Set this true if you want Vuforia to track beyond the target.
        parameters.useExtendedTracking = false;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");
        this.vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Load the trackable objects from the Assets file, and give them meaningful names
        VuforiaTrackables targetsFreightFrenzy = this.vuforia.loadTrackablesFromAsset("FreightFrenzy");
        targetsFreightFrenzy.get(0).setName("Blue Storage");
        targetsFreightFrenzy.get(1).setName("Blue Alliance Wall");
        targetsFreightFrenzy.get(2).setName("Red Storage");
        targetsFreightFrenzy.get(3).setName("Red Alliance Wall");

        // Start tracking targets in the background
        targetsFreightFrenzy.activate();

        waitForStart();

        while (opModeIsActive())
        {
            for (VuforiaTrackable trackable : targetsFreightFrenzy)
            {
                double targetX, targetY;
                if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible())
                {
                    targetPose = ((VuforiaTrackableDefaultListener)trackable.getListener()).getVuforiaCameraFromTarget();

                    // if we have a target, process the "pose" to determine the position of the target relative to the robot.
                    if (targetPose != null)
                    {
                        String targetName  = trackable.getName();
                        VectorF trans = targetPose.getTranslation();

                        // Extract the X & Y components of the offset of the target relative to the robot
                        targetX = trans.get(0) / MM_PER_INCH; // Image X axis
                        targetY = trans.get(2) / MM_PER_INCH; // Image Z axis

                        telemetry.addLine(targetName + ", robot @ (" + targetX + "," + targetY + ")");
                    }
                }
            }
        }
    }
}
