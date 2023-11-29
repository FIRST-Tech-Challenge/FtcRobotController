package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

@Autonomous
public class CameraAutoTest extends LinearOpMode {

    private static final boolean USE_WEBCAM = true;
    private TfodProcessor tfod;
    private VisionPortal visionPortal;

    double TicsPerRevolution = 537.6;
    double Circumference = 11.87;
    double TPI = TicsPerRevolution / Circumference;
    double StrafeTPI = 50.2512563;
    private float clawOpen = 1, clawClose = 0;
    private Servo claw;
    private DcMotor motorLeft, motorLeft2,
            motorRight, motorRight2, motorIntake;
    private WebcamName webcamName       = null;

    private void initTfod() {

        tfod = new TfodProcessor.Builder()
                .build();

        VisionPortal.Builder builder = new VisionPortal.Builder();
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }
        builder.addProcessor(tfod);
        visionPortal = builder.build();
    }

    //TPI is ticks per inches
    int ConvertInchesToTicks(double inches) {
        return (int) (inches * TPI);
    }

    int StrafeInchesToTicks(double inches) {
        return (int) (inches * StrafeTPI);
    }
    @Override
    public void runOpMode() throws InterruptedException {
        initTfod();
    }

}
