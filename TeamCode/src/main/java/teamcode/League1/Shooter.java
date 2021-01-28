package teamcode.League1;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import java.util.Timer;

import teamcode.common.AbstractOpMode;
import teamcode.common.Constants;
import teamcode.common.Debug;
import teamcode.common.Localizer;
import teamcode.common.MecanumDriveTrain;
import teamcode.common.Utils;
import teamcode.test.revextensions2.ExpansionHubEx;
import teamcode.test.revextensions2.RevBulkData;

public class Shooter {
    private static final double INDEXER_EXTENDED_POS = 1;
    private static final double INDEXER_RETRACTED_POS = 0.65;
    /*
    Electronics Schematic:
    1x Motor Roller
    2x flywheel motor
    1xIndexing Actuator
    1x secondary Indexer valve
    this schematic assumes the through gravity indexer
     */

    DcMotor  roller, flywheel, upperRoller;
    Servo indexer;
    ExpansionHubEx hub;

    public Shooter(HardwareMap hardwareMap){
        roller = hardwareMap.dcMotor.get("LowerIntakeMotor");
        upperRoller = hardwareMap.dcMotor.get("UpperIntakeMotor");
        flywheel = hardwareMap.dcMotor.get("Shooter");
        indexer = hardwareMap.servo.get("Indexer");
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        indexer.setPosition(INDEXER_RETRACTED_POS);
        Debug.log(indexer.getPosition());
    }


    /**
     * generic intake for Tele Op
     * @param power power to run the intake
     */
    public void intake(double power){
        roller.setPower(power);
    }

    /**
     * runs intake for specified number of millis, for Auto
     * @param power power to run the intake
     * @param millis millis intake should run
     */
    public void intake(double power, long millis){
        roller.setPower(power);
        Utils.sleep(millis);
        roller.setPower(0);
    }


    public void shoot(int rings){
        try {
            flywheel.setPower(-0.97);//calibrate this
            Thread.sleep(1500);
            for(int i = 0; i < rings; i++) { //make this parametric?
                indexer.setPosition(INDEXER_EXTENDED_POS);
                Thread.sleep(250);
                indexer.setPosition(INDEXER_RETRACTED_POS);
                Thread.sleep(250);
            }
            flywheel.setPower(0);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }



    /**
     * uses time and power on the indexer
     */
    public void ghettoShoot(){
        flywheel.setPower(0.97);
        try {
            Thread.currentThread().sleep(1000);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        indexer.setPosition(0.6);
        try {
            Thread.currentThread().sleep(5020);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        indexer.setPosition(0.5);
        flywheel.setPower(0);

    }

    /*
     * Direction     Heading
     * 0             -90
     * 90            0
     * 180           90
     * 270           180
     *
     */
    private double headingToDirectionRads(double heading){
        return heading + (Math.PI / 2.0);
    }

    /**
     * keeping this code for later, DO NOT DELETE OR CALL
     */

//    private void autoOrient(){
//        double distanceAwayX = Constants.GOAL_POSITION.x - Localizer.thisLocalizer().getCurrentPosition().x;
//        double distanceAwayY = Constants.GOAL_POSITION.y - Localizer.thisLocalizer().getCurrentPosition().y;
//        double angleDiff = Math.atan2(distanceAwayY, distanceAwayX);
//        drive.rotate(headingToDirectionRads(angleDiff), 0.4);
//        double distanceAway = Math.sqrt(Math.pow(distanceAwayX, 2) + Math.pow(distanceAwayY, 2));
//    }



    //TODO add endgame functions and game specific electronics
    //Wobble Grabber
}
