package com.bravenatorsrobotics.freightfrenzy;

import com.bravenatorsrobotics.common.drive.MecanumDrive;
import com.bravenatorsrobotics.common.operation.AutonomousMode;
import com.bravenatorsrobotics.common.vision.TensorFlowObjectDetector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

@Autonomous(name="Autonomous")
public class Auto extends AutonomousMode<MecanumDrive> {

    private Config config;

    public Auto() { super(new Specifications()); }

    @Override
    public void OnInitialize() {
        config = new Config(hardwareMap.appContext); // Get the saved configuration for later

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


    // Warehouse Code (compatible for both sides with the 'movementModifier')
    private void RunWarehouse(int movementModifier) {
        // Drive to the alliance shipping hub

        // Deliver the preloaded block (make sure the robot has this)

        // Strafe and park fully in the warehouse
    }

    // Storage Unit Code (compatible for both sides with the 'movementModifier')
    private void RunStorageUnit(int movementModifier) {
        // Drive to the turn table

        // Spin the turn table

        // Drive Forward

        // Drive along side the barcode until the team block is detected

        // Drop the preload box into the wobble goal
    }

    @Override
    public void OnStop() {
        robot.drive.Stop();
    }
}
