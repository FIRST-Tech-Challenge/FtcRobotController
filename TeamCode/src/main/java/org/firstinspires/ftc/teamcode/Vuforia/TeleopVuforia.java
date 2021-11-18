package org.firstinspires.ftc.teamcode.Vuforia;

import androidx.core.math.MathUtils;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.CompBotV3.CompBotV3;
import org.firstinspires.ftc.teamcode.CompBotV3.CompBotV3Attachments;

@TeleOp(name="Viridian Competition Teleop Vuforia")
public class TeleopVuforia extends OpMode {
    CompBotV3Attachments r = new CompBotV3Attachments();
    double initialHeading, error;
    boolean headingReset = false;

    final double MM_PER_INCH = 25.40 ;   //  Metric conversion

    private static final String VUFORIA_KEY =
            "AQXJtf3/////AAABmRNtqRHOi01ztX4zAd+KuPdOXWfS1wa0VbmgatYieybPLp6S7F+PpEIxktdGYJhBEd9SBRox+HmVviMD3XQdouhnWR8NtrjrlFXN63orJIBQHEhUC8vZTbO1Sq/EsAELA4VhNOHmERzTXhJ4Kz8h4Cy4tJr192IiMB7W02Czjts055dln8QTMQPXqXgFU4qeY7mxKrpFco1pKT/OFCdwVHWWvkozJBXEIujp/eqjknmpKgrlPQPRkFOzyLRHHCybs/R9Agw6EDiVGgE92XCU/lFOaU0U92R/WlWoFGh/LlyHgqlFg6N/SskENvsnX1j7tbCF68Bz78meWiaRYSktMJRgshC1Fk3G7EnmMKaCPFMJ";

    VuforiaLocalizer vuforia    = null;
    OpenGLMatrix targetPose     = null;
    VuforiaTrackables targetsFreightFrenzy;

    @Override
    public void init() {
        r.init(hardwareMap);
        // Set up camera
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        // Turn off Extended tracking.  Set this true if you want Vuforia to track beyond the target.
        parameters.useExtendedTracking = false;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");
        this.vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Load the trackable objects from the Assets file, and give them meaningful names
        targetsFreightFrenzy = this.vuforia.loadTrackablesFromAsset("FreightFrenzy");
        targetsFreightFrenzy.get(0).setName("Blue Storage");
        targetsFreightFrenzy.get(1).setName("Blue Alliance Wall");
        targetsFreightFrenzy.get(2).setName("Red Storage");
        targetsFreightFrenzy.get(3).setName("Red Alliance Wall");

        // Start tracking targets in the background
        targetsFreightFrenzy.activate();
    }

    @Override
    public void loop() {
        double y = gamepad1.left_stick_y, x = -1*gamepad1.left_stick_x, turn = -1*gamepad1.right_stick_x;

        // Deadzone
        y = (Math.abs(y)>0.05 ? y : 0);
        x = (Math.abs(x)>0.05 ? x : 0);
        turn = (Math.abs(turn)>0.05 ? turn : 0);

        // Power adjust
        y *= (gamepad1.right_bumper?0.4:1);
        x *= (gamepad1.right_bumper?0.4:1);
        turn *= (gamepad1.right_bumper?0.4:1);

        if(Math.abs(y) > Math.abs(x)) {
            x = 0;
        } else {
            y = 0;
        }
        if (Math.abs(turn) < 0.1 && Math.abs(x) > 0 && Math.abs(y) > 0) {
            if(!headingReset) {
                initialHeading = r.imu.getHeading();
                headingReset = true;
            } else {
                error = r.imu.getHeading() - initialHeading;
                turn = CompBotV3.corrCoeff*error;
            }
        } else {
            headingReset = false;
        }
        r.fl.setPower(MathUtils.clamp(y+x+turn ,-1,1));
        r.fr.setPower(MathUtils.clamp(-(y-x-turn),-1,1));
        r.bl.setPower(MathUtils.clamp(y-x+turn,-1,1));
        r.br.setPower(MathUtils.clamp(-(y+x-turn),-1,1));

        r.intake.setPower((gamepad1.a?1:0) - (gamepad1.b?1:0));
        r.lift.setPower((gamepad1.dpad_up?1:0) - (gamepad1.dpad_down?1:0));
        r.spin.setPower(gamepad1.right_trigger-gamepad1.left_trigger);
        r.bucket.setPower(gamepad1.left_bumper?-1:1);

        int a = r.lift.getCurrentPosition();

        if(gamepad1.right_stick_button) {
            r.imu.reset();
        }

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
                    // target range is based on distance from robot position to origin (right triangle).
                    double targetRange = Math.hypot(targetX, targetY);

                    // target bearing is based on angle formed between the X axis and the target range line
                    double targetBearing = Math.toDegrees(Math.asin(targetX / targetRange));

                    telemetry.addLine(targetName + ", robot @ (" + targetX + "," + targetY + "), range = "+targetRange+", angle = "+targetBearing);
                }
            }
        }
        telemetry.update();

    }

    @Override
    public void stop() {
        r.stop();
    }
}
