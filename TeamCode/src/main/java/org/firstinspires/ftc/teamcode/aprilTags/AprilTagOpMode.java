package org.firstinspires.ftc.teamcode.aprilTags;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class AprilTagOpMode extends LinearOpMode {

    AprilTagDetection aprilTagDetection = new AprilTagDetection();
    public void runOpMode()
    {
        aprilTagDetection.Setup(hardwareMap, telemetry);

        telemetry.setMsTransmissionInterval(50);

        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */

        while (!isStarted() && !isStopRequested()) {

            telemetry.addLine("Value is:" + aprilTagDetection.GetDistanceAwayFromTheBackdrop());
            telemetry.update();

            //Orientation rot = Orientation.getOrientation(currentDetections.get().pose.R, AxesReference.INTRINSIC, AxesOrder.YXZ, AngleUnit.DEGREES);

            // Debugging
            /*if (!(currentDetections.isEmpty())) {
                tagToTelemetry(currentDetections.get(0));
                telemetry.update();
            }*/
        }
        /* You wouldn't have this in your autonomous, this is just to prevent the sample from ending */
        while (opModeIsActive()) {sleep(20);}
    }
}
