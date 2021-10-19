package org.firstinspires.ftc.teamcode;

import com.bravenatorsrobotics.drive.MecanumDrive;
import com.bravenatorsrobotics.operation.AutonomousMode;
import com.bravenatorsrobotics.vision.TensorFlowObjectDetector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

@Autonomous(name="Autonomous")
public class Auto extends AutonomousMode<MecanumDrive> {

    public static final int DUCK_LEFT_THRESHOLD = 200;
    public static final int DUCK_RIGHT_THRESHOLD = 550;

    private Config config;

    private enum DuckPosition {
        LEFT, RIGHT, CENTER, UNKNOWN
    }

    private DuckPosition duckPosition = DuckPosition.UNKNOWN;

    public Auto() { super(new Specifications()); }

    @Override
    public void OnInitialize() {
        config = new Config(hardwareMap.appContext);

        TensorFlowObjectDetector objectDetector = new TensorFlowObjectDetector(this, "Webcam 1");

        telemetry.addData("Status", "Updating Recognitions");
        telemetry.update();

        // Detect for the gold brick
        objectDetector.Initialize();
        while(!isStarted()) {
            objectDetector.UpdateRecognitions();

            Recognition duckRecognition = objectDetector.GetFirstRecognitionByType(TensorFlowObjectDetector.ObjectType.DUCK);
            if (duckRecognition != null) {
                if (duckRecognition.getLeft() < DUCK_LEFT_THRESHOLD)
                    duckPosition = DuckPosition.LEFT;
                else if (duckRecognition.getLeft() > DUCK_RIGHT_THRESHOLD)
                    duckPosition = DuckPosition.RIGHT;
                else
                    duckPosition = DuckPosition.CENTER;
            }

            telemetry.addData("Duck Position", duckPosition.toString());
            telemetry.update();

            sleep(5);
        }

        if(duckPosition == DuckPosition.UNKNOWN) {
            if(specifications.debugModeEnabled) {
                telemetry.addLine("ERROR: Could not detect the duck!!!");
                telemetry.update();
            }

            duckPosition = DuckPosition.LEFT;
        }
    }

    @Override
    public void OnStart() {
        double movementModifier = config.allianceColor == Config.AllianceColor.RED ? 1 : -1;

        switch (config.startingPosition) {
            case WAREHOUSE:
                break;
            case STORAGE_UNIT:
                break;
        }
    }

    @Override
    public void OnStop() {
        robot.drive.Stop();
    }
}
