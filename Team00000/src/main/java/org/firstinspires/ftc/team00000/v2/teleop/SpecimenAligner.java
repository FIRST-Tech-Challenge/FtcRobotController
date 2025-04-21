package org.firstinspires.ftc.team00000.v2.teleop;


import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.team00000.v2.util.AngleServoController;
import org.firstinspires.ftc.team00000.v2.vision.ColorVisionSubsystem;

@TeleOp(name = "Specimen Aligner Test", group = "Test")
public class SpecimenAligner extends LinearOpMode {

    @Override public void runOpMode() {

        WebcamName cam      = hardwareMap.get(WebcamName.class, "Webcam 1");
        Servo      wristSrv = hardwareMap.get(Servo.class,          "wrist_drive");

        ColorVisionSubsystem vision = new ColorVisionSubsystem(cam);
        AngleServoController servo  = new AngleServoController(wristSrv, telemetry);

        FtcDashboard.getInstance().startCameraStream(vision.getPortal(), 0);

        waitForStart();

        while (opModeIsActive()) {

            vision.update();

            if (vision.hasTarget()) {
                servo.update(vision.getAngleErrorToVertical());
            }

            telemetry.addData("Target?", vision.hasTarget());
            telemetry.addData("Angle°",  "%.1f", vision.getAngle());
            telemetry.addData("Area",    "%.0f", vision.getArea());
            telemetry.update();

            sleep(10);                    // ~100 Hz loop, AngleServoController throttles itself
        }

        vision.stop();
    }
}