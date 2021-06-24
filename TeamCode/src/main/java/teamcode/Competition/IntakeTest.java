package teamcode.Competition;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import teamcode.common.AbstractOpMode;
@Disabled
@Autonomous(name="Intake")
public class IntakeTest extends AbstractOpMode {
    Shooter shoot;
    @Override
    protected void onInitialize() {
        shoot = new Shooter(hardwareMap);
    }

    @Override
    protected void onStart() {
        while(opModeIsActive()){
            shoot.intake(1);
        }
    }

    @Override
    protected void onStop() {

    }
}
