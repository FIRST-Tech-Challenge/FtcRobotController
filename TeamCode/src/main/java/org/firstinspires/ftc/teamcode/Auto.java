package org.firstinspires.ftc.teamcode;

import com.bravenatorsrobotics.drive.AbstractDrive;
import com.bravenatorsrobotics.operation.AutonomousMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Autonomous")
public class Auto extends AutonomousMode {

    public Auto() { super(new Specifications()); }

    @Override
    public void OnInitialize() {

    }

    @Override
    public void OnStart() {
        robot.drive.TurnDegrees(0.75, 90, AbstractDrive.TurnDirection.CLOCKWISE);
    }

    @Override
    public void OnStop() {
        robot.drive.Stop();
    }
}
