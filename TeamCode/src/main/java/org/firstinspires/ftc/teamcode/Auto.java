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
        config = new Config(hardwareMap.appContext); // Get the saved configuration for later

        ScanBarcode(); // Scan for the duck's position
    }

    @Override
    public void OnStart() {
        // 1 if on blue alliance -1 if on red alliance
        int movementModifier = config.allianceColor == Config.AllianceColor.BLUE ? 1 : -1;

        // Call the appropriate method and pass in the movement modifier
        switch (config.startingPosition) {
            case WAREHOUSE:
                RunWarehouse(movementModifier);
                break;
            case STORAGE_UNIT:
                RunStorageUnit(movementModifier);
                break;
        }
    }

    private void ScanBarcode() {
        telemetry.addData("Status", "Updating Recognitions");
        telemetry.update();

        TensorFlowObjectDetector objectDetector = new TensorFlowObjectDetector(this, "Webcam 1");

        // Detect for the duck
        objectDetector.Initialize(); // Initialize the object detector
        while(!isStarted()) { // Loop until the start button is pressed
            objectDetector.UpdateRecognitions(); // Scan for all game objects

            // Get the duck's position and determine which spot it's in using the threshold values.
            Recognition duckRecognition = objectDetector.GetFirstRecognitionByType(TensorFlowObjectDetector.ObjectType.DUCK);
            if (duckRecognition != null) {
                if (duckRecognition.getLeft() < DUCK_LEFT_THRESHOLD)
                    duckPosition = DuckPosition.LEFT;
                else if (duckRecognition.getLeft() > DUCK_RIGHT_THRESHOLD)
                    duckPosition = DuckPosition.RIGHT;
                else
                    duckPosition = DuckPosition.CENTER;
            }

            // Push the duck's estimated position to the drivers.
            telemetry.addData("Duck Position", duckPosition.toString());
            telemetry.update();

            sleep(5); // Allow the CPU to ramp-down
        }

        // If the duck wasn't found, assume it's in the left position and print a warning.
        if(duckPosition == DuckPosition.UNKNOWN) {
            if(specifications.debugModeEnabled) {
                telemetry.addLine("WARNING: Could not detect the duck!!!");
                telemetry.update();
            }

            duckPosition = DuckPosition.LEFT;
        }
    }

    // Warehouse Code (compatible for both sides with the 'movementModifier')
    private void RunWarehouse(int movementModifier) {
        // Drive to the alliance shipping hub

        // Deliver the preloaded block (make sure the robot has this)

        // Strafe and park fully in the warehouse
    }

    // Storage Unit Code (compatible for both sides with the 'movementModifier')
    private void RunStorageUnit(int movementModifier) {
        // Drive to the alliance shipping hub

        // Deliver the preloaded block (make sure the robot has this)

        // Drive to the turn-table

        // Spin the turn-table

        // Park fully in the Storage Unit
    }

    @Override
    public void OnStop() {
        robot.drive.Stop();
    }
}
