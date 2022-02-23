package org.wheelerschool.robotics.old.lib.vision;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.android.util.Size;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraCaptureRequest;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraException;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

import java.util.Objects;

@TeleOp(name="vuforia testing", group="testing")
public class NewVuforiaTestClass extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor motorFrontRight;
    private DcMotor motorBackRight;
    private DcMotor motorFrontLeft;
    private DcMotor motorBackLeft;

    VuforiaLocalizer vuforia;

    WebcamName webcamName;

    private static final String VUFORIA_KEY =
            "Ab7oYbj/////AAABmUWCVRgoZkuuu7bkLaZWztV7Ce6KfyDyGfPgSRuFHm3URYp9CqsXMGsvKUxaji17TFgwS/zkC66Lt1zEtXINDE58AeSA8f2gNcdr+pQO8hZvRmoIUjmniZ5Fr7IuPuHoz2cWgcPN8H8EJOviayojof5lroZ4A6HI9PXHowRW8GPkjFRJDOhY6GpapILDQRu4UxPNKqM6+diTjb2KkJn8XtGE5vxPdqRAS4dSPy9yRrCiAnwzTMRy+DaELRDsOl1sgaEOMzjlv1919iSxBUwQUTRRWeg13l14BemOfTgpCmpC2DbzoAujIBKolyeys0yXTLhI4ETJSOICAKp3wvhFwxpKFH1LCs+vIuTrxx+pACtK";


    @Override
    public void runOpMode() throws InterruptedException {

        webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = webcamName;

        this.vuforia = ClassFactory.getInstance().createVuforia(parameters);

        try {
            telemetry.addData("test", Objects.requireNonNull(vuforia.getCamera()).createCaptureRequest(1, new Size(4, 4), 5));
            CameraCaptureRequest cc = Objects.requireNonNull(vuforia.getCamera()).createCaptureRequest(1, new Size(4, 4), 5);


        } catch (CameraException e) {
            e.printStackTrace();
        }


        telemetry.addData("Status", "Initialized");
        telemetry.update();



        // Declare our motors
        // Make sure your ID's match your configuration in the Driver Hub
        motorFrontRight = hardwareMap.get(DcMotor.class, "motorFrontRight"); // port #3

        motorFrontLeft = hardwareMap.get(DcMotor.class, "motorFrontLeft"); // port #2

        motorBackRight = hardwareMap.get(DcMotor.class, "motorBackRight"); // port #1

        motorBackLeft = hardwareMap.get(DcMotor.class, "motorBackLeft"); // port #0
        waitForStart();

        runtime.reset();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            if(gamepad1.a){

            }
            telemetry.addData("controller", gamepad1.a);
            // It's useful to have a variable for each motor to save its power level for telemetry
            // ... but not used in this example
            double frontRightMotorPower;
            double backRightMotorPower;
            double frontLeftMotorPower;
            double backLeftMotorPower;


            // The game pad stick input feeds the variable
            frontRightMotorPower = -(gamepad1.right_stick_y + gamepad1.right_stick_x);
            backRightMotorPower = -(gamepad1.right_stick_y - gamepad1.right_stick_x);
            frontLeftMotorPower = gamepad1.right_stick_y - gamepad1.right_stick_x;
            backLeftMotorPower = gamepad1.right_stick_y + gamepad1.right_stick_x;


            // Set the DC motor power to our variable
            motorFrontRight.setPower(frontRightMotorPower);
            motorBackRight.setPower(backRightMotorPower);
            motorFrontLeft.setPower(frontLeftMotorPower);
            motorBackLeft.setPower(backLeftMotorPower);
            telemetry.update();
        }
    }
}

