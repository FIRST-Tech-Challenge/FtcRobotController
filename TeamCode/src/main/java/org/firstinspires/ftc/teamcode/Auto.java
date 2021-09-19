package org.firstinspires.ftc.teamcode;

import com.bravenatorsrobotics.drive.AbstractDrive;
import com.bravenatorsrobotics.drive.FourWheelDrive;
import com.bravenatorsrobotics.drive.MecanumDrive;
import com.bravenatorsrobotics.drive.TwoWheelDrive;
import com.bravenatorsrobotics.operation.AutonomousMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Autonomous(name="Autonomous")
public class Auto extends AutonomousMode<MecanumDrive> {

    private Config config;

    public Auto() { super(new Specifications()); }

    @Override
    public void OnInitialize() {
        config = new Config(hardwareMap.appContext);
    }

    @Override
    public void OnStart() {
        double movementModifier = config.allianceColor == Config.AllianceColor.RED ? 1 : -1;

        switch (config.startingPosition) {
            case SHIPPING_HUB:
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
