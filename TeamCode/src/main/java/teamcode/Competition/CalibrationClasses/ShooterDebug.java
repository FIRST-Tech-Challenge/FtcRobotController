package teamcode.Competition.CalibrationClasses;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.openftc.revextensions2.ExpansionHubMotor;

import teamcode.common.AbstractOpMode;

@Disabled
@Autonomous(name="shoot")
public class ShooterDebug extends AbstractOpMode {

    ExpansionHubMotor flywheel;
    Servo indexer;

    @Override
    protected void onInitialize() {
        flywheel = hardwareMap.get(ExpansionHubMotor.class, "Shooter");
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        indexer =hardwareMap.servo.get("Indexer");
        indexer.setPosition(0.4);
    }

    @Override
    protected void onStart() {
        while(opModeIsActive()){
            if(gamepad1.x){

            }
        }
    }

    @Override
    protected void onStop() {

    }
}
