package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.SleeveDetection;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

import org.firstinspires.ftc.robotcore.internal.vuforia.VuforiaPoseMatrix;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import org.openftc.easyopencv.OpenCvInternalCamera;

@Autonomous(name="Autonomie bazata dreapta", group="Linear Opmode")
public class AutonomieBazataDreapta extends GlobalScope2023 {
    private SleeveDetection sleeveDetection;
    private OpenCvCamera camera;
    private String webcamName = "Webcam 1";
    int stateBratz=0;
    final double cmtoinch=0.3937;
    int pos = 1;
    void ParkingMaster()
    {
        ///codul pentru scanare pls?

    }

    void FlippyAction()
    {
        rot.setDirection(Servo.Direction.REVERSE);
        rot.setPosition(0.47+0.18);
    }
    void UnFlippyAction()
    {
        //rot.setDirection(Servo.Direction.FORWARD);
        rot.setPosition(0.47+0.06);
    }
    void InchideCleste()
    {
        c1.setDirection(Servo.Direction.FORWARD);
        c2.setDirection(Servo.Direction.FORWARD);
        c1.setPosition(0.55);
        c2.setPosition(0.45);
    }
    void DeschideCleste()
    {
        c1.setDirection(Servo.Direction.REVERSE);
        c2.setDirection(Servo.Direction.REVERSE);
        c1.setPosition(0.95);
        c2.setPosition(0.05);
    }
    int ok = 0;
    void RidicaBratz(int pozitie)
    {
        mb1.setTargetPosition(pbrat[pozitie]);
        mb2.setTargetPosition(pbrat[pozitie]);
        mb1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        mb2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        mb1.setPower(0.4);
        mb2.setPower(0.4);
        //mb2.setPower(0.22);
        //sj.setDirection(Servo.Direction.REVERSE);
        if(pozitie!=3)
            sj.setPosition(psjauto[pozitie]);

    }
    @Override
    public void runOpMode() throws InterruptedException {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, webcamName), cameraMonitorViewId);
        sleeveDetection = new SleeveDetection();
        camera.setPipeline(sleeveDetection);

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(320,240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {}
        });

        while (!isStarted()) {
            telemetry.addData("ROTATION: ", sleeveDetection.getPosition());
            telemetry.update();
        }
        /// AUTONOMIE PE PARTEA STANGA A ALIANTEI!
        Initialise();
        mb1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mb2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        waitForStart();
        int poz = sleeveDetection.getPosition();
        TrajectorySequence p1 = drive.trajectorySequenceBuilder(new Pose2d(36.00, -60.00, Math.toRadians(90.00)))
                .lineTo(new Vector2d(11.00, -60.00))
                .lineTo(new Vector2d(11.00, -10.00))
                .build();
        TrajectorySequence p2=drive.trajectorySequenceBuilder(p1.end())
                .lineTo(new Vector2d(35.00, -10.00))
                .build();
        TrajectorySequence p3=drive.trajectorySequenceBuilder(p1.end())
                .lineTo(new Vector2d(62.00, -10.00))
                .build();
        drive.setPoseEstimate(p1.start());
        drive.followTrajectorySequence(p1);
        if(poz==2)
            drive.followTrajectorySequence(p2);
        else if(poz==3)
            drive.followTrajectorySequence(p3);
        while(opModeIsActive())   ;


    }
}
