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

@Autonomous(name="Autonomie bazata", group="Linear Opmode")
public class Autonomie2023 extends GlobalScope2023 {
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
        TrajectorySequence bazatus = drive.trajectorySequenceBuilder(new Pose2d(-36.00, -60.00, Math.toRadians(90.00)))
                .lineTo(new Vector2d(-16, -60.00))
                .lineTo(new Vector2d(-12.00, -10.00))
                .lineTo(new Vector2d(-36.00, -10.00))
                .build();
        TrajectorySequence b2=drive.trajectorySequenceBuilder(bazatus.end().plus(new Pose2d(0,0,Math.toRadians(90))))
                .lineTo(new Vector2d(-37.5, 0.50))
                //.lineTo(new Vector2d(-34.00, 0.00))
                .build();
        TrajectorySequence b3=drive.trajectorySequenceBuilder(b2.end())
                .lineTo(new Vector2d(-37.50, -11.00))
                .lineTo(new Vector2d(-53.50, -11.00))
                .build();
        TrajectorySequence b4=drive.trajectorySequenceBuilder(b3.end())
                .lineTo(new Vector2d(-38, -11))
                //.lineTo(new Vector2d(-52.00, 3))
                .lineTo(new Vector2d(-38, 0.5))
                .build();
        TrajectorySequence p2=drive.trajectorySequenceBuilder(b4.end())
                .lineTo(new Vector2d(-40.00, 0.5))
                .lineTo(new Vector2d(-39.00, -10.5))
                .build();
        TrajectorySequence p1=drive.trajectorySequenceBuilder(p2.end())
                .lineTo(new Vector2d(-62.00, -10.5))
                .build();
        TrajectorySequence p3=drive.trajectorySequenceBuilder(p2.end())
                .lineTo(new Vector2d(-12.00, -10.5))
                .build();

        waitForStart();
        drive.setPoseEstimate(new Pose2d());
        int poz = sleeveDetection.getPosition();
        rot.setPosition(0.47);
        InchideCleste();
        sleep(600);
        RidicaBratz(3);
        /**while(opModeIsActive() && mb1.isBusy() && mb2.isBusy()) ;
        mb1.setPower(0);
        mb2.setPower(0);
        mb1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        mb2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);*/
        sj.setPosition(0.47);
        sleep(200);
        drive.setPoseEstimate(bazatus.start());
        drive.followTrajectorySequence(bazatus);
        drive.turn(Math.toRadians(90));
        drive.followTrajectorySequence(b2);
        sj.setDirection(Servo.Direction.REVERSE);
        sj.setPosition(psj[5]-0.03);
        sleep(500);
        DeschideCleste();
        sleep(200);
        mb1.setTargetPosition(-240);
        mb2.setTargetPosition(-240);
        mb1.setPower(0.3);
        mb2.setPower(0.3);
        sj.setPosition(0.47);
        /*while(opModeIsActive() && mb1.isBusy() && mb2.isBusy()) ;
        mb1.setPower(0);
        mb2.setPower(0);
        mb1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        mb2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);*/
        drive.followTrajectorySequence(b3);
        InchideCleste();
        sleep(500);
        RidicaBratz(4);
        sj.setDirection(Servo.Direction.REVERSE);
        sj.setPosition(0.42);
        sleep(500);
        RidicaBratz(3);
        /*while(opModeIsActive() && mb1.isBusy() && mb2.isBusy()) ;
        mb1.setPower(0);
        mb2.setPower(0);
        mb1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        mb2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);*/
        sleep(400);
        sleep(400);
        sj.setPosition(0.55);
        FlippyAction();

        sleep(200);
        drive.followTrajectorySequence(b4);
        sleep(600);
        sj.setDirection(Servo.Direction.REVERSE);
        sj.setPosition(0.47);
        sleep(650);
        DeschideCleste();
        //Parcare
        drive.followTrajectorySequence(p2);
        if (poz == 1)
            drive.followTrajectorySequence(p1);
        else if (poz == 3)
            drive.followTrajectorySequence(p3);

        /*
        sj.setPosition(0.55);
        sleep(2000);
        stateBratz=5;
        sj.setDirection(Servo.Direction.REVERSE);
        sj.setPosition(psj[stateBratz]);
        mb1.setTargetPosition(posbrat[stateBratz]);
        mb2.setTargetPosition(posbrat[stateBratz]);
        mb1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        mb2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        mb1.setPower(0.2);
        mb2.setPower(0.2);
        sleep(500);
        stateBratz=0;
        sj.setPosition(psj[stateBratz]);
        sleep(500);
        mb1.setTargetPosition(posbrat[stateBratz]);
        mb2.setTargetPosition(posbrat[stateBratz]);

        mb1.setPower(0.1);
        mb2.setPower(0.1);
        sleep(5000);

        /**int poz = sleeveDetection.getPosition();
        telemetry.addData("LEPOSITION ", poz);
        telemetry.update();

        if (poz == 1)
        {
            telemetry.addData("AM TRECUT: ", "PRIN STANGA");
            telemetry.update();
            //MoveAuto(43, 'L', 0.5);
            MotorSS.setPower(-0.6);
            MotorFD.setPower(0.6);
            sleep(2100);
            StopMotorsRoti();

        }
        else if (poz == 3) {
            //MoveAuto(43, 'R', 0.5);
            MotorSS.setPower(0.6);
            MotorFD.setPower(-0.6);
            sleep(2100);
            StopMotorsRoti();
        }

        MoveAuto(75, 'F', 0.5);
        */
        while(opModeIsActive())   ;


    }
}
