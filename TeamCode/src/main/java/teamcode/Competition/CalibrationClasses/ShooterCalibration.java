package teamcode.Competition.CalibrationClasses;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import teamcode.Competition.Shooter;
import teamcode.common.AbstractOpMode;


@Disabled
@TeleOp(name="Shooter calibration")
public class ShooterCalibration extends AbstractOpMode {
    Shooter shooter;

    @Override
    protected void onInitialize() {
        shooter = new Shooter(hardwareMap);
    }

    @Override
    protected void onStart() {
        //shooter.setFlywheelPower(0.9);
        shooter.setFlywheelVelocity(1500, AngleUnit.DEGREES);
        while(opModeIsActive()){
            if(gamepad1.x){
                shooter.progressBanana();
                try {
                    Thread.sleep(250);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
                shooter.progressBanana();
            }
            telemetry.addData("flywheel velocity", shooter.getFlywheelVelocity());
            telemetry.update();
        }
    }

    @Override
    protected void onStop() {

    }
}
