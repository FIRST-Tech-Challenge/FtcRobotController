package org.firstinspires.ftc.teamcode;
import androidx.annotation.NonNull;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;


// JOSHUANOTE: Change Auto name here.
@Config
@Autonomous(name = "AutonomousShell1", group = "Autonomous")
public class AutonomousShell1 extends LinearOpMode {
    private VisionPortal visionPortal;
    private BlueRightProcessor colorMassDetectionProcessor;
    // JOSHUANOTE: initialization of servos and hardware mapping is here.
    public class Serv {
        private Servo Serv;


        public Serv(HardwareMap hardwareMap) {
            Serv = hardwareMap.get(Servo.class, "Example");
        }
        // JOSHUANOTE: Servo action functions listed here.
        public class ExampleServo implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                Serv.setPosition(0);
                return false;
            }
        }
        public Action ExampleAction() {
            return new ExampleServo();
        }


        public class ExampleServo2 implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                Serv.setPosition(0);
                return false;
            }
        }
        public Action ExampleAction2() {
            return new ExampleServo2();
        }
    }
//-------------------------------------------------------------------------------------------


    @Override
    public void runOpMode() {
        colorMassDetectionProcessor = new BlueRightProcessor();
        // JOSHUANOTE: Init for the auto is here. sets color and sets up camera.
        colorMassDetectionProcessor.setDetectionColor(false); //false is blue, true is red
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam"))
                .addProcessors(colorMassDetectionProcessor)
                .build();
        // JOSHUANOTE: Position sets the cords and heading of the spot you start.
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, Math.toRadians(0)));
        Serv gate = new Serv(hardwareMap);


        // JOSHUANOTE: I believe the 2 means the default that gets changed by the processor, not sure.
        // vision here that outputs position
        int visionOutputPosition = 2;


        // JOSHUANOTE: Here is where the trajectories are intitialized and defined.


        Action ExampleTrajectory;
        Action ExampleTrajectory2;
        Action wait;


        ExampleTrajectory = drive.actionBuilder(drive.pose)
                .build();


        ExampleTrajectory2 = drive.actionBuilder(drive.pose)
                .build();


        wait = drive.actionBuilder(drive.pose)
                .waitSeconds(1)
                .build();






        while (!isStopRequested() && !opModeIsActive()) {
            int position = visionOutputPosition;
            telemetry.addData("Currently Recorded Position: ", colorMassDetectionProcessor.getPropLocation());
            telemetry.addData("Camera State: ", visionPortal.getCameraState());
            telemetry.addData("Position during Init", position);
            FtcDashboard.getInstance().startCameraStream(colorMassDetectionProcessor, 30);
            telemetry.update();
        }
        // JOSHUANOTE: This is where the camera input is taken and turned into variable usage.
            BlueRightProcessor.PropPositions recordedPropPosition = colorMassDetectionProcessor.getPropLocation();
        switch (recordedPropPosition) {
            case LEFT:
                visionOutputPosition = 1;
                break;
            case MIDDLE:
                visionOutputPosition = 2;
                break;
            case RIGHT:
                visionOutputPosition = 3;
                break;
        }


        int startPosition = visionOutputPosition;
        telemetry.addData("Starting Position", startPosition);
        telemetry.update();
        waitForStart();


        if (isStopRequested()) return;


        Action trajectoryPurpleChosen;
        Action trajectoryYellowChosen;
        Action trajectoryCloseOutChosen;
        // JOSHUANOTE: this is the logic for picking the route
        if (startPosition == 2) {


        } else if (startPosition == 3) {


        } else {


        }
        // Actions are set as values assigned to the final actions run in the program for all outcomes.




        Actions.runBlocking(
                new SequentialAction(
                        // JOSHUANOTE: This is where you put the final set of actions.
                )
        );
        // Bad Grammar here, leaving it as a warning.
        telemetry.addLine("Didnt Closed Camera.");
        telemetry.update();
    }
}
