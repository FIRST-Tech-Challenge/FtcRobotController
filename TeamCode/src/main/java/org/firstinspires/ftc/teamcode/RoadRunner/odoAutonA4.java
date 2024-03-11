package org.firstinspires.ftc.teamcode.RoadRunner;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.MainFolderComp.A4.A4helpers.A4bluePipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.Objects;

@Config
@Autonomous(name = "odoAutonA4L", group = "odo")
public class odoAutonA4 extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    private OpenCvWebcam webcam;
    A4bluePipeline pipeline = new A4bluePipeline();
    public class AutoY {
        private Servo AutoY;

        public AutoY(HardwareMap hardwareMap) {
            AutoY = hardwareMap.get(Servo.class, "autoy");
        }

        public class AutoYdown implements Action {
            private boolean initialized = false;

            @Override
            public boolean run( TelemetryPacket packet) {
                if (!initialized) {
                    AutoY.setPosition(0);
                    initialized = true;
                }

                double pos = AutoY.getPosition();
                packet.put("AutoYpos", pos);
                if (pos > 0) {
                    return true;
                } else {
                    return false;
                }
            }
        }
        public Action AutoYDown() {
            return new AutoYdown();
        }

        public class AutoYup implements Action {
            private boolean initialized = false;

            @Override
            public boolean run( TelemetryPacket packet) {
                if (!initialized) {
                    AutoY.setPosition(0.8);
                    initialized = true;
                }

                double pos = AutoY.getPosition();
                packet.put("AutoYpos", pos);
                if (pos < 0.8) {
                    return true;
                } else {
                    return false;
                }
            }
        }
        public Action AutoYUp() {
            return new AutoYup();
        }
    }

    public class AutoP {
        private Servo AutoP;

        public AutoP(HardwareMap hardwareMap) {
            AutoP = hardwareMap.get(Servo.class, "autop");
        }

        public class AutoPdown implements Action {
            private boolean initialized = false;

            @Override
            public boolean run( TelemetryPacket packet) {
                if (!initialized) {
                    AutoP.setPosition(0);
                    initialized = true;
                }

                double pos = AutoP.getPosition();
                packet.put("AutoPpos", pos);
                if (pos > 0.1) {
                    return true;
                } else {
                    return false;
                }
            }
        }
        public Action AutoPDown() {
            return new AutoPdown();
        }

        public class AutoPup implements Action {
            private boolean initialized = false;

            @Override
            public boolean run( TelemetryPacket packet) {
                if (!initialized) {
                    AutoP.setPosition(0.5);
                    initialized = true;
                }

                double pos = AutoP.getPosition();
                packet.put("AutoPpos", pos);
                if (pos < 0.5) {
                    return true;
                } else {
                    return false;
                }
            }
        }
        public Action AutoPUp() {
            return new AutoPup();
        }
    }

    @Override
    public void runOpMode() {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, Math.toRadians(90)));
        AutoY autoy = new AutoY(hardwareMap);
        AutoP autop = new AutoP(hardwareMap);

        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam"));

        webcam.setPipeline(pipeline);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {

                // start streaming the camera
                webcam.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);

            }

            @Override
            public void onError(int errorCode) {
                telemetry.addLine("camera pipeline error");
            }
        });

        // vision pipeline is pipeline.getPosition()

        Action trajectoryActionLeft;
        Action trajectoryActionMiddle;
        Action trajectoryActionRight;
        Action trajectoryActionNull;

        trajectoryActionLeft = drive.actionBuilder(drive.pose)
                .setTangent(Math.toRadians(65))
                .lineToX(75)
                .waitSeconds(5)
                .build();
        trajectoryActionMiddle = drive.actionBuilder(drive.pose)
                .strafeTo(new Vector2d(44.5, 25))

//                .lineToX(18)
//                .waitSeconds(3)
//                .setTangent(Math.toRadians(0))
//                .lineToXSplineHeading(46, Math.toRadians(180))
//                .waitSeconds(3)
                .build();
        trajectoryActionRight = drive.actionBuilder(drive.pose)
                .setTangent(0)
                .splineTo(new Vector2d(48, 48), Math.PI / 2)
                .waitSeconds(3)
                .build();
        trajectoryActionNull = drive.actionBuilder(drive.pose)
                .build();

        // actions that need to happen on init; for instance, a claw tightening.
        Actions.runBlocking(autop.AutoPDown());


        while (!isStopRequested() && !opModeIsActive()) {
            telemetry.addData("Prop Location: ", pipeline.getLocation());
            telemetry.update();
        }

        waitForStart();

        if (isStopRequested()) return;

        Action trajectoryActionChosen = trajectoryActionNull;

        if (Objects.equals(pipeline.getLocation(), "left")) {
            trajectoryActionChosen = trajectoryActionMiddle;
        } else if (Objects.equals(pipeline.getLocation(), "middle")) {
            trajectoryActionChosen = trajectoryActionMiddle;
        } else if ((Objects.equals(pipeline.getLocation(), "right"))) {
            trajectoryActionChosen = trajectoryActionRight;
        }

        Actions.runBlocking(
                new SequentialAction(
                        trajectoryActionChosen,
                        autop.AutoPUp()

                )
        );
        requestOpModeStop();
    }

}
