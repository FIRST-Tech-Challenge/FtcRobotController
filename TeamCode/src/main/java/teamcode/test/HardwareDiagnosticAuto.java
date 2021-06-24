package teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;

import teamcode.common.AbstractOpMode;
import teamcode.common.Debug;
import teamcode.common.Localizer;
import teamcode.common.MecanumDriveTrain;
import teamcode.common.Point;
import teamcode.common.Utils;
import teamcode.common.Vector2D;

@Disabled
@Autonomous(name="HardwareDiagnostic")
public class HardwareDiagnosticAuto extends AbstractOpMode {
    Localizer localizer;
    MecanumDriveTrain drive;
    private final double POWER = 0.5;
    @Override
    protected void onInitialize() {
        localizer = new Localizer(hardwareMap, new Vector2D(0,0), 0);
        drive = new MecanumDriveTrain(hardwareMap);

    }

    @Override
    protected void onStart() {

        //FIXED
        try {
            drive.setPower(POWER, 0, 0, 0); //good
            Thread.sleep(1000);
            drive.setPower(0, POWER, 0, 0); //good
            Thread.sleep(1000);
            drive.setPower(0, 0, POWER, 0); //bad
            Thread.sleep(1000);
            drive.setPower(0, 0, 0, POWER); //bad
            Thread.sleep(1000);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }



        //        DcMotor[] motors = drive.getMotors();
//        motors[0].setPower(POWER);
//        Debug.log("Front Left: " + motors[0].getPower());
//        Debug.log("FL Port Num: " + motors[0].getPortNumber());
//        Debug.log("FL RunMode: " + motors[0].getMode());
//        Utils.sleep(5000);
//        motors[0].setPower(0);
//
//        motors[1].setPower(POWER);
//        Debug.log("Front Right: " + motors[1].getPower());
//        Debug.log("FR Port Num: " + motors[1].getPortNumber());
//        Debug.log("FR RunMode: " + motors[1].getMode());
//        Utils.sleep(5000);
//        motors[1].setPower(0);
//
//        motors[2].setPower(POWER);
//        Debug.log("Back Left: " + motors[2].getPower());
//        Debug.log("BL Port Num: " + motors[2].getPortNumber());
//        Debug.log("BL RunMode: " + motors[2].getMode());
//        Utils.sleep(5000);
//        motors[2].setPower(0);
//
//        motors[3].setPower(POWER);
//        Debug.log("Back Right: " + motors[3].getPower());
//        Debug.log("BR Port Num: " + motors[3].getPortNumber());
//        Debug.log("BR RunMode: " + motors[3].getMode());
//        Utils.sleep(5000);
//        motors[3].setPower(0);
    }

    @Override
    protected void onStop() {

    }
}
