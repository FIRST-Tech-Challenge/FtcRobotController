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
import teamcode.common.Debug;
import teamcode.common.Localizer;
import teamcode.common.MecanumDriveTrain;
import teamcode.common.Point;
import teamcode.common.PurePursuit.CurvePoint;
import teamcode.common.PurePursuit.MovementVars;
import teamcode.common.PurePursuit.PurePursuitMovement;
import teamcode.common.Vector2D;
import teamcode.test.AyushVision.NumRings;
import teamcode.test.AyushVision.RingDetectionPipeline;

@Autonomous(name="NewAuto")
public class LeagueThreeAutoGood extends AbstractOpMode {

    MecanumDriveTrain drivetrain;
    Localizer localizer;
    PurePursuitMovement movement;
    ArrayList<CurvePoint> allPoints;
    OpenCvWebcam webcam;
    Shooter shooter;
    WobbleArm arm;
    NumRings rings;
    Thread driveThread, armThread, driveCommandThread;
    boolean isPurePursuit;

    @Override
    protected void onInitialize() {
        localizer = new Localizer(hardwareMap, new Vector2D(0, 0), 0); //calibrate this
        //movement = new PurePursuitMovement(localizer);
        drivetrain = new MecanumDriveTrain(hardwareMap);
        shooter = new Shooter(hardwareMap);
        arm = new WobbleArm(hardwareMap, true);
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam"), cameraMonitorViewId);
        RingDetectionPipeline pipeline = new RingDetectionPipeline();
        webcam.setPipeline(pipeline);
        isPurePursuit = true;
//        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
//            @Override
//            public void onOpened() {
//                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPSIDE_DOWN); //specify cam orientation and calibrate the resolution
//            }
//        });
        driveThread = new Thread(){
            @Override
            public void run(){
                driveSequence();
            }
        };
        armThread = new Thread(){
            public void run(){
                try {
                    Thread.sleep(5000);

                } catch (InterruptedException e) {
                    e.printStackTrace();
                }

            }
        };
//////        driveCommandThread = new Thread(){
//////            public void run(){
//////                while(opModeIsActive()) {
//////                    if(isPurePursuit) {
//////                        drivetrain.setPowerPurePursuit(new Vector2D(MovementVars.movementX, MovementVars.movementY), MovementVars.movementTurn);
//////                    }
//////                }
////            }
//        };
//        while(!opModeIsActive()){
//            rings = pipeline.getRings();
//            telemetry.addData("Rings:" ,rings);
//            telemetry.addData("Localizer pos", localizer.getCurrentPosition());
//            telemetry.addData("localizer rads", localizer.getGlobalRads());
//            telemetry.update();
//        }
    }

    private void driveSequence() {
        //movement.isActive = true;
//        while(movement.isActive && AbstractOpMode.currentOpMode().opModeIsActive()) {
//            movement.followCurve(new CurvePoint(0, 0,0.5,0,1, 0,0),
//                    new CurvePoint(-24, 4,0.5,0,1, 0,0), 0);
//            telemetry.addData("current pos", localizer.getCurrentPosition());
//            telemetry.addData("current rot", localizer.getGlobalRads());
//            telemetry.addData("followMe",  movement.getFollowMe());
//            telemetry.update();
//        }

        shooter.setFlywheelVelocity(1710 * 0.855, AngleUnit.DEGREES);
        try {
            Thread.sleep(1500);
            shooter.progressBanana();
            Thread.sleep(250);
            shooter.progressBanana();
            shooter.setFlywheelVelocity(1710 * 0.87, AngleUnit.DEGREES);
            Thread.sleep(1500);
            shooter.progressBanana();
            Thread.sleep(250);
            shooter.progressBanana();
            shooter.setFlywheelVelocity(1710 * 0.84, AngleUnit.DEGREES);
            Thread.sleep(1500);
            shooter.progressBanana();
            Thread.sleep(250);
            shooter.progressBanana();
            Thread.sleep(250);
            drivetrain.setPower(0.75, 0.75, -0.75, -0.75);
            Thread.sleep(2000);
            drivetrain.setPower(0,0,0,0);
            shooter.setFlywheelVelocity(0,AngleUnit.RADIANS);
            arm.runToPosition(-4000, 0.5);
            arm.adjustClaw();
            arm.runToRelativePosition(-3000, 0.5);
            arm.runToRelativePosition(-4000, 0.5);




        } catch (InterruptedException e) {
            e.printStackTrace();
        }
            while(opModeIsActive());
//        while(!movement.robotIsNearPoint(new Point(12, -2), 2)){
//            movement.goToPosition(12, -2, 0.5, 0, 0);
//            telemetry.addData("current pos", localizer.getCurrentPosition());
//            telemetry.addData("current rot", localizer.getGlobalRads());
//            telemetry.update();
//
//            if(movement.robotIsNearPoint(new Point(12, -2), 2)){
//                brake();
//                break;
//            }
//        }


//        movement.isActive = true;
//        Debug.log("next point");
//        while(movement.isActive && AbstractOpMode.currentOpMode().opModeIsActive()) {
//            movement.followCurve(new CurvePoint(localizer.getCurrentPosition().x, localizer.getCurrentPosition().y,1,0.5,3, 0,0),
//                    new CurvePoint(-12, 96,1,0.5,3, 0,0), 0);
//            telemetry.addData("current pos", localizer.getCurrentPosition());
//            telemetry.addData("current rot", localizer.getGlobalRads());
//            telemetry.update();
//        }


    }

    private void brake() {
        MovementVars.movementX = 0;
        MovementVars.movementY = 0;
        MovementVars.movementTurn = 0;
    }

    @Override
    protected void onStart() {
        driveThread.start();
        //armThread.start();
        //driveCommandThread.start();
        while(opModeIsActive());
    }

    @Override
    protected void onStop() {
        driveThread.interrupt();
        armThread.interrupt();
        driveCommandThread.interrupt();
    }
}
