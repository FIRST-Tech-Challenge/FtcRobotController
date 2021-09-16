package teamcode.offSeason;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.ExpansionHubServo;

import teamcode.common.AbstractOpMode;
import teamcode.common.Debug;

public class MockShooter {
    //TODO calibrate these values
    private static final double INDEXER_RETRACTED_VALUE = 0;
    private static final double INDEXER_EXTENDED_VALUE = 1;
    /*
        Hardware Components:
        1x Shooter Motor
        1x Indexer (Servo)
         */
    ExpansionHubMotor flywheel;
    ExpansionHubServo indexer;
    DcMotor intakeFront, intakeBack;


    public MockShooter(HardwareMap hardwareMap){
        flywheel = (ExpansionHubMotor) hardwareMap.dcMotor.get("Flywheel");
        indexer = (ExpansionHubServo) hardwareMap.servo.get("Indexer");
        intakeFront = hardwareMap.dcMotor.get("intakeFront");
        intakeBack = hardwareMap.dcMotor.get("intakeBack");
        resetHardware();

    }

    
    private void intake(double power){
        intakeFront.setPower(power);
        intakeBack.setPower(-power);
    }

    private void resetHardware() {
        indexer.setPosition(INDEXER_RETRACTED_VALUE);
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        PIDCoefficients coefficient = new PIDCoefficients(1,0.01, 0.1);
        flywheel.setPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, coefficient);
    }

    public void mockShoot(double power, int rings){
        flywheel.setPower(power);
        try {
            Thread.sleep(1000);
            for(int i = 0; i < rings; i++){
                indexer.setPosition(INDEXER_EXTENDED_VALUE);
                Thread.sleep(250);
                indexer.setPosition(INDEXER_RETRACTED_VALUE);
                Thread.sleep(250);
            }
            flywheel.setPower(0);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }
    //pi radians = 180 degrees, also unitless
    public void mockShootBetter(double velocity, int rings){

        try {
            for(int i = 0; i < rings; i++){
                flywheel.setVelocity(velocity, AngleUnit.RADIANS);
                while(Math.abs(flywheel.getVelocity(AngleUnit.RADIANS) - velocity) > (Math.PI / 30.0) && AbstractOpMode.currentOpMode().opModeIsActive());
                indexer.setPosition(INDEXER_EXTENDED_VALUE);
                Thread.sleep(250);
                indexer.setPosition(INDEXER_RETRACTED_VALUE);
                Thread.sleep(100);
            }
            flywheel.setPower(0);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }

}