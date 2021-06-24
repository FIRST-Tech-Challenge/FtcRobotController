package teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import teamcode.Competition.Shooter;
import teamcode.common.AbstractOpMode;
import teamcode.common.Utils;

@Autonomous(name="startShoot")
public class autoConcept extends AbstractOpMode {

    Shooter shooter;
    @Override
    protected void onInitialize() {
        shooter = new Shooter(hardwareMap);
    }

    @Override
    protected void onStart() {
        shooter.setFlywheelVelocity(2400, AngleUnit.RADIANS);
        while(opModeIsActive()) {
            if(gamepad1.x){
                shooter.progressBanana();
                Utils.sleep(250);
                shooter.progressBanana();
            }
            telemetry.addData("velocity", shooter.getFlywheelVelocity());
            telemetry.update();
        }

        //shooter.shoot(3, 30000);

    }

    @Override
    protected void onStop() {

    }
}
