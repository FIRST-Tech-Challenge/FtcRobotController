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

        menu.addOption("Alliance Color", Config.AllianceColor.class, Config.allianceColor);

        menu.setGamepad(gamepad1);
        menu.setTelemetry(telemetry);

        waitForStart();

        while(opModeIsActive()) {
            menu.displayMenu();

            switch (menu.getCurrentChoiceOf("Alliance Color")) {
                case "RED":
                    Config.allianceColor = Config.AllianceColor.RED;
                    break;
                case "BLUE":
                    Config.allianceColor = Config.AllianceColor.BLUE;
                    break;
            }

            sleep(50);
        }

        config.Save();
    }

}
