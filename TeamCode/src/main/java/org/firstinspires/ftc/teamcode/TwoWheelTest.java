package org.firstinspires.ftc.teamcode;

import com.bravenatorsrobotics.drive.AbstractDrive;
import com.bravenatorsrobotics.drive.TwoWheelDrive;
import com.bravenatorsrobotics.operation.AutonomousMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="TwoWheelTest")
public class TwoWheelTest extends AutonomousMode<TwoWheelDrive> {

    public TwoWheelTest() {
        super(new TwoWheelSpecs());
    }

    @Override
    public void OnInitialize() {

    }

    @Override
    public void OnStart() {
        super.robot.drive.TurnDegrees(0.25, 90, AbstractDrive.TurnDirection.CLOCKWISE);
    }

    @Override
    public void OnStop() {

    }
}
