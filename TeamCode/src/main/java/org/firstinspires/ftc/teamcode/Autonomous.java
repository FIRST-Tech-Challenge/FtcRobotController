package org.firstinspires.ftc.teamcode;

import com.bravenatorsrobotics.operation.AutonomousMode;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="Autonomous")
public class Autonomous extends AutonomousMode {

    public Autonomous() { super(new Specifications()); }

    @Override
    public void OnInitialize() {

    }

    @Override
    public void OnStart() {
        robot.drive.DriveByInches(0.75, 24);
    }

    @Override
    public void OnStop() {

    }
}
