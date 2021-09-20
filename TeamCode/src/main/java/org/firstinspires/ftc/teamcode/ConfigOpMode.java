package org.firstinspires.ftc.teamcode;

import com.bravenatorsrobotics.core.SimpleMenu;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Configuration", group="Config")
public class ConfigOpMode extends LinearOpMode {

    private static final SimpleMenu menu = new SimpleMenu("Configuration Menu");

    @Override
    public void runOpMode() {
        Config config = new Config(hardwareMap.appContext);

        menu.clearOptions();

        menu.addOption("Alliance Color", Config.AllianceColor.class, config.allianceColor);
        menu.addOption("Starting Position", Config.StartingPosition.class, config.startingPosition);

        menu.setGamepad(gamepad1);
        menu.setTelemetry(telemetry);

        waitForStart();

        while(opModeIsActive()) {
            menu.displayMenu();

            switch (menu.getCurrentChoiceOf("Alliance Color")) {
                case "RED":
                    config.allianceColor = Config.AllianceColor.RED;
                    break;
                case "BLUE":
                    config.allianceColor = Config.AllianceColor.BLUE;
                    break;
            }

            switch (menu.getCurrentChoiceOf("Starting Position")) {
                case "STORAGE_UNIT":
                    config.startingPosition = Config.StartingPosition.STORAGE_UNIT;
                    break;
                case "WAREHOUSE":
                    config.startingPosition = Config.StartingPosition.WAREHOUSE;
                    break;
            }

            sleep(50);
        }

        config.Save();
    }

}
