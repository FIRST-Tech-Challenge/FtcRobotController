package teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import teamcode.Competition.Shooter;
import teamcode.common.AbstractOpMode;

@TeleOp(name="Shooter Calibrator")
public class ShooterTest extends AbstractOpMode {

    Shooter shooter;

    @Override
    protected void onInitialize() {
        shooter = new Shooter(hardwareMap);
    }

    @Override
    protected void onStart() {
        shooter.setFlywheelVelocity(1710 * 0.845, AngleUnit.DEGREES);
        while(opModeIsActive()){
            telemetry.addData("flywheel velocity", shooter.getFlywheelVelocity());
            telemetry.update();
            if(gamepad1.x){
                shooter.progressBanana();
                try {
                    Thread.sleep(250);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
                shooter.progressBanana();
            }else if(gamepad1.dpad_down){
                shooter.setFlywheelVelocity(shooter.getFlywheelRadians() - 10, AngleUnit.DEGREES);
                try {
                    Thread.sleep(100);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }else if(gamepad1.dpad_up){
                shooter.setFlywheelVelocity(shooter.getFlywheelRadians() + 10, AngleUnit.DEGREES);
                try {
                    Thread.sleep(100);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }
        }
    }

    @Override
    protected void onStop() {

    }
}
