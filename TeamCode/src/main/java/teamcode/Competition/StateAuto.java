package teamcode.Competition;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.ArrayList;

import teamcode.common.AbstractOpMode;
import teamcode.common.Localizer;
import teamcode.common.MecanumDriveTrain;
import teamcode.common.PurePursuit.CurvePoint;
import teamcode.common.PurePursuit.PurePursuitMovement;
import teamcode.common.Utils;
import teamcode.common.Vector2D;
import teamcode.test.AyushVision.NumRings;
import teamcode.test.AyushVision.RingDetectionPipeline;


@Autonomous(name="Auto")
public class StateAuto extends AbstractOpMode {

    Localizer localizer;
    MecanumDriveTrain drive;
    Shooter shooter;
    OpenCvWebcam webcam;
    WobbleArm arm;
    NumRings rings;
    Thread driveThread, armThread, driveCommandThread;


    @Override
    protected void onInitialize() {
        localizer = new Localizer(hardwareMap, new Vector2D(0, 0), 0); //calibrate this
        drive = new MecanumDriveTrain(hardwareMap, localizer);
        shooter = new Shooter(hardwareMap);
        arm = new WobbleArm(hardwareMap, true);
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam"), cameraMonitorViewId);
        RingDetectionPipeline pipeline = new RingDetectionPipeline();
        webcam.setPipeline(pipeline);
                webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPSIDE_DOWN); //specify cam orientation and calibrate the resolution
            }
        });
        while(!opModeIsActive()){
            rings = pipeline.getRings();
            telemetry.addData("rings", rings);
            telemetry.update();
        }
    }

    @Override
    protected void onStart() {
        webcam.stopStreaming();
        localizer.start();
//        drive.moveToPosition(new Vector2D(0,2), 6.0, 0);
//        drive.moveToRotation(Math.toRadians(-6), -0.25);
//        drive.moveToPosition(new Vector2D(18,2), 12.0, 0);
//        drive.moveToRotation(Math.toRadians(1), 0.25);
//        //drive.moveToRotation(Math.toRadians(10.0), 0.25);
//        shooter.setFlywheelVelocity(1710 * 0.855, AngleUnit.DEGREES);
//        try {
//            Thread.sleep(750);
//            shooter.progressBanana();
//            Thread.sleep(250);
//            shooter.progressBanana();
//            shooter.setFlywheelVelocity(1710 * 0.87, AngleUnit.DEGREES);
//            Thread.sleep(750);
//            shooter.progressBanana();
//            Thread.sleep(250);
//            shooter.progressBanana();
//            shooter.setFlywheelVelocity(1710 * 0.835, AngleUnit.DEGREES);
//            Thread.sleep(750);
//            shooter.progressBanana();
//            Thread.sleep(250);
//            shooter.progressBanana();
//        }catch(InterruptedException e){
//            e.printStackTrace();
//        }
//        shooter.setFlywheelVelocity(0,AngleUnit.RADIANS);
//        drive.moveToRotation(0, -0.25);
//        drive.moveToPosition(new Vector2D(11,97), 24.0, 0);
//
//        //drive.moveToPosition(new Vector2D(17,94), 6.0, 0);
//
//        if(rings == NumRings.ONE){
//            drive.moveToRotation(Math.PI / 2.0, 0.5);
//        }else if(rings == NumRings.FOUR){
//            drive.moveToRotation(Math.PI, 0.5);
//        }
//        arm.runToPosition(-4000, 1);
//        arm.adjustClaw();
//        Utils.sleep(100);
//        arm.runToPosition(4000, 0.5);
//
//        //Utils.sleep(250);
//
//        if(rings == NumRings.ZERO) {
//
//            drive.moveToPosition(new Vector2D(11, 115), 12, 0);
//        }else{
//            drive.moveToRotation(Math.toRadians(5), -0.5);
//            drive.moveToPosition(new Vector2D(11, 72), 12, 0);
//        }
//
//
//        drive.moveToPosition(new Vector2D(-11,72), 21, 0);
//
//        if(rings != NumRings.ZERO){
////            drive.moveToPosition(new Vector2D(localizer.getCurrentState().getPosition().getX(),8), 18.0, 0);
//            drive.moveToRotation(Math.toRadians(20), 0.5);
////            arm.runToPosition(-1000, 0.5);
////            drive.moveToPosition(new Vector2D(-5, 8), 6.0, Math.PI / 2.0);
//
//            //drive.moveToPosition(new Vector2D(11,88), 21, 0);
//            drive.moveToRotation(Math.toRadians(5), -0.5);
//            if(rings == NumRings.ONE){
//                shooter.intake(1.0);
//            }
//            drive.moveToPosition(new Vector2D(-11,39), 24, Math.toRadians(5));
//            shooter.intake(1.0);
//
//            //drive.moveToPosition(new Vector2D(-15,34), 12, 0);
//            //Utils.sleep(1000);
//            drive.moveToRotation(Math.toRadians(9), 0.5);
//            Utils.sleep(1000);
//            shooter.shoot(3, 1440);
//            //drive.moveToRotation(Math.toRadians(3), -0.5);
////            if(rings == NumRings.FOUR) {
////                drive.moveToPosition(new Vector2D(-19, 12), 6, 0);
////                drive.moveToPosition(new Vector2D(-19, 36), 12, 0);
////                drive.moveToRotation(Math.toRadians(13), 0.5);
////                shooter.shoot(3, 1450);
////            }
//        }
//
//        drive.moveToPosition(new Vector2D(-19,69), 24, 0);
//        shooter.zero();
        while(opModeIsActive());



        //drive.moveToRotation(Math.toRadians(0), -0.25);

    }

    @Override
    protected void onStop() {
        localizer.stopThread();
    }
}
