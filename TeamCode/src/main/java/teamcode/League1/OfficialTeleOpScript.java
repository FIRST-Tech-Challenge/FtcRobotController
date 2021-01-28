package teamcode.League1;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import teamcode.common.AbstractOpMode;
import teamcode.common.MecanumDriveTrain;
import teamcode.common.Vector2D;

@TeleOp(name="tele op")
public class OfficialTeleOpScript extends AbstractOpMode {

    Shooter shooter;
    MecanumDriveTrain drive;
    Thread driveThread;
    Thread shootThread;

    private static final double INTAKE_POWER = 0.7;
    private static final double SPRINT_LINEAR_MODIFIER = 0.75;
    private static final double NORMAL_LINEAR_MODIFIER = 0.5;
    private static final double SPRINT_ROTATIONAL_MODIFIER = 0.75;
    private static final double NORMAL_ROTATIONAL_MODIFIER = 0.5;

    @Override
    protected void onInitialize() {
        shooter = new Shooter(hardwareMap);
        drive = new MecanumDriveTrain(hardwareMap);
        driveThread = new Thread(){
            public void run(){
                while(opModeIsActive()){
                    driveUpdate();
                }
            }
        };
        shootThread = new Thread(){
            public void run(){
                while(opModeIsActive()){
                    shootUpdate();
                }
            }
        };
    }

    private void shootUpdate() {
        if(gamepad1.x) {
            shooter.shoot(5);
        }else if(gamepad1.right_trigger > 0.3){
            shooter.intake(INTAKE_POWER);
        }else if(gamepad1.left_trigger > 0.3){
            shooter.intake(-INTAKE_POWER);
        }else {
            shooter.intake(0);
        }
    }

    private void driveUpdate() {
        if(gamepad1.left_bumper){
            drive.setPower(new Vector2D( gamepad1.left_stick_x * SPRINT_LINEAR_MODIFIER, gamepad1.left_stick_y * SPRINT_LINEAR_MODIFIER),
                    gamepad1.right_stick_x * SPRINT_ROTATIONAL_MODIFIER);
        }else{
            drive.setPower(new Vector2D( gamepad1.left_stick_x * NORMAL_LINEAR_MODIFIER, gamepad1.left_stick_y * NORMAL_LINEAR_MODIFIER),
                    gamepad1.right_stick_x * NORMAL_ROTATIONAL_MODIFIER);
        }

    }

    @Override
    protected void onStart() {
        driveThread.start();
        shootThread.start();
        while(opModeIsActive());
    }

    @Override
    protected void onStop() {

    }
}
