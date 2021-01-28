package teamcode.League1.CalibrationClasses;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import teamcode.League1.Shooter;
import teamcode.common.AbstractOpMode;


@TeleOp(name="shoot some shit")
public class ShooterCalibration extends AbstractOpMode {
    Shooter shooter;

    @Override
    protected void onInitialize() {
        shooter = new Shooter(hardwareMap);
    }

    @Override
    protected void onStart() {
        shooter.shoot(3);
        while(opModeIsActive());
    }

    @Override
    protected void onStop() {

    }
}
