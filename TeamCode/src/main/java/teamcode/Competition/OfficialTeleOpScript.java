package teamcode.Competition;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import teamcode.common.AbstractOpMode;
import teamcode.common.Debug;
import teamcode.common.MecanumDriveTrain;
import teamcode.common.Utils;
import teamcode.common.Vector2D;


//15:40
@TeleOp(name="tele op")
public class OfficialTeleOpScript extends AbstractOpMode {

    Shooter shooter;
    MecanumDriveTrain drive;
    Thread driveThread, driverTwoThread;
    Thread shootThread;
    WobbleArm WGG;
    BNO055IMU imu;
    boolean isShooting;

    private static final double INTAKE_POWER = 1.0;
    private static final double SPRINT_LINEAR_MODIFIER = 1.0;
    private static final double NORMAL_LINEAR_MODIFIER = 1.0;
    private static final double SPRINT_ROTATIONAL_MODIFIER = 1.0;
    private static final double NORMAL_ROTATIONAL_MODIFIER = 0.5;
    private boolean isSprint;
    private ArmState armState;

    @Override
    protected void onInitialize() {
        shooter = new Shooter(hardwareMap);
        drive = new MecanumDriveTrain(hardwareMap);
        WGG = new WobbleArm(hardwareMap, false);
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        isSprint = true;
        armState = armState.RETRACTED;
        //Initialize IMU parameters
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.RADIANS;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        //parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu.initialize(parameters);
        isShooting = false;

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
//        driverTwoThread = new Thread(){
//            public void run(){
//                while(opModeIsActive()){
//                    driverTwoUpdate();
//                }
//            }
//        };

    }

    private void driverTwoUpdate() {
        if(gamepad2.x){
            WGG.adjustClaw();
            Utils.sleep(250);
        }else if (gamepad2.right_trigger > 0.3) {
            while(gamepad2.right_trigger > 0.3){
                shooter.intake(INTAKE_POWER);
            }
            shooter.intake(0);
        } else if (gamepad2.left_trigger > 0.3) {
            while(gamepad2.left_trigger > 0.3){
                shooter.intake(-INTAKE_POWER);
            }
            shooter.intake(0);
            //WGG.runToPosition(WobbleConstants.SCORING_POSITION, 0.5);
        } else if (gamepad2.dpad_up) {
            if(armState == ArmState.RETRACTED) {
                WGG.runToPosition(-2500, 1);
                armState = ArmState.ENDGAME;
            }else if(armState == ArmState.GRABBING){
                WGG.runToRelativePosition(-2500, 1);
                armState = ArmState.ENDGAME;
            }

        }else if(gamepad2.dpad_down){
            if(armState == ArmState.RETRACTED) {
                WGG.runToPosition(-4200, 1);
                armState = ArmState.GRABBING;
            }else if(armState == ArmState.ENDGAME){
                WGG.runToRelativePosition(-4200, 1);
                armState = ArmState.GRABBING;
            }
        }else if(gamepad2.dpad_left){
//            while(gamepad2.dpad_left){
//                WGG.setPower(0.5);
//            }
//            WGG.setPower(0);
            WGG.runToRelativePosition(0, 1);
        }else if(gamepad2.left_bumper){
            while(gamepad2.left_bumper){
                WGG.setPower(-0.5);
            }
            WGG.setPower(0);
            WGG.resetEncoder();
        }else if(gamepad2.right_bumper){
            while(gamepad2.right_bumper){
                WGG.setPower(0.5);
            }
            WGG.setPower(0);
            WGG.resetEncoder();
        }else if(gamepad2.a){
            WGG.resetEncoder();
        }
        if(Math.abs(WGG.motorEncoder.getCurrentPosition()) < 100){
            armState = ArmState.RETRACTED;
        }

    }

    private enum ArmState {
        RETRACTED, ENDGAME, GRABBING
    }

    private void shootUpdate() {
        if (gamepad1.x) {
            shooter.shoot(3, 1450); //1450
            //shooter.shoot(1, 1330);
            //shooter.shoot(1, 1330);
        } else if (gamepad1.right_trigger > 0.3) {
            while(gamepad1.right_trigger > 0.3) {
                shooter.intake(INTAKE_POWER);
            }
            shooter.intake(0);
        } else if (gamepad1.left_trigger > 0.3) {
            while(gamepad1.left_trigger > 0.3) {
                shooter.intake(-INTAKE_POWER);
            }
            shooter.intake(0);
        } else if (gamepad1.left_bumper) {
            shooter.setPower(0.86);
        } else if (gamepad1.b) {
            shooter.progressBanana();
            Utils.sleep(250);
            shooter.progressBanana();
        } else if(gamepad1.a){
            shooter.zero();
        }

    }

    private static final double ROTATE_DPAD = 0.3;
    private static final double LINEAR_DPAD = 0.5;
    private void driveUpdate() {
        if(gamepad1.right_stick_button){
            drive.setPower(new Vector2D( gamepad1.left_stick_x * NORMAL_LINEAR_MODIFIER, gamepad1.left_stick_y * NORMAL_LINEAR_MODIFIER),
                    gamepad1.right_stick_x * SPRINT_ROTATIONAL_MODIFIER);
        }else if(gamepad1.dpad_left){
            while(gamepad1.dpad_left) {
                drive.setPower(-ROTATE_DPAD, ROTATE_DPAD, -ROTATE_DPAD, ROTATE_DPAD);
            }
            drive.setPower(0,0,0,0);
        }else if(gamepad1.dpad_right){
            while(gamepad1.dpad_right){
                drive.setPower(ROTATE_DPAD, -ROTATE_DPAD,ROTATE_DPAD,-ROTATE_DPAD);
            }
            drive.setPower(0,0,0,0);
        }else if (gamepad1.dpad_up) {
            while(gamepad1.dpad_up){
                drive.setPower(LINEAR_DPAD, LINEAR_DPAD, LINEAR_DPAD, LINEAR_DPAD);
            }
            drive.zero();
        }else if(gamepad1.dpad_down) {
            while(gamepad1.dpad_down){
                drive.setPower(-LINEAR_DPAD, -LINEAR_DPAD, -LINEAR_DPAD, -LINEAR_DPAD);
            }
            drive.zero();
        }else{
            drive.setPower(new Vector2D( gamepad1.left_stick_x * NORMAL_LINEAR_MODIFIER, gamepad1.left_stick_y * NORMAL_LINEAR_MODIFIER),
                    gamepad1.right_stick_x * NORMAL_ROTATIONAL_MODIFIER);
        }

    }

    @Override
    protected void onStart() {
        //WGG.runToPosition(WobbleConstants.RETRACTED_POSITION, 0.5);
        driveThread.start();
        shootThread.start();
        //driverTwoThread.start();
        while(opModeIsActive());
    }

    @Override
    protected void onStop() {

    }
}
