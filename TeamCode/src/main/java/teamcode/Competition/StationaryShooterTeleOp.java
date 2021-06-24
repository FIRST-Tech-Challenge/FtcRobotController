package teamcode.Competition;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import teamcode.common.AbstractOpMode;

@Disabled
@TeleOp(name="Shooter Tele")
public class StationaryShooterTeleOp extends AbstractOpMode {

    private static final double INTAKE_POWER = 0.7;
    Thread shooterThread;
    Shooter shooter;

    //CRServo intake;

    @Override
    protected void onInitialize() {
        //intake = hardwareMap.crservo.get("Intake Servo");
        shooter = new Shooter(hardwareMap);
        shooterThread = new Thread(){
            @Override
            public void run(){
                while(opModeIsActive()){
                    shooterUpdate();
                }
            }
        };
    }

    private void shooterUpdate() {
        if(gamepad1.x) {
            shooter.shoot(3, 0.95);
        }else if(gamepad1.right_trigger > 0.3){
            shooter.intake(INTAKE_POWER);
        }else if(gamepad1.left_trigger > 0.3){
            shooter.intake(-INTAKE_POWER);
        }else if(gamepad1.left_bumper){
            //intake.setPower(1);
        }else if(gamepad1.right_bumper){
            //intake.setPower(-1);
        } else{
            shooter.intake(0);
        }
    }

    @Override
    protected void onStart() {
        shooterThread.start();
        while(opModeIsActive());
    }

    @Override
    protected void onStop() {

    }
}
