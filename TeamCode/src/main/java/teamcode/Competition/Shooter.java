package teamcode.Competition;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.ExpansionHubMotor;

import teamcode.Competition.CalibrationClasses.PIDController;
import teamcode.common.AbstractOpMode;
import teamcode.common.Constants;
import teamcode.common.Localizer;
import teamcode.common.Utils;


public class Shooter {

    private static final double RPM_TOLERANCE = 100;
    /*
    Electronics Schematic:
    1x Motor Roller
    2x flywheel motor
    1xIndexing Actuator
    1x secondary Indexer valve
    this schematic assumes the through gravity indexer
     */

    ExpansionHubMotor roller, flywheel, upperRoller;
    Servo indexer;
    ExpansionHubEx hub;
    double flywheelPreviousPosition;
    long previousTime;
    boolean isFlywheelInit;
    PIDController pidShoot;

    public Shooter(HardwareMap hardwareMap){
        roller = (ExpansionHubMotor)hardwareMap.get("LowerIntakeMotor");
        upperRoller = (ExpansionHubMotor)hardwareMap.dcMotor.get("UpperIntakeMotor");
        flywheel = hardwareMap.get(ExpansionHubMotor.class, "Shooter");
        indexer = hardwareMap.servo.get("Indexer");
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        indexer.setPosition(0.4);
        isFlywheelInit = false;
        hub = hardwareMap.get(ExpansionHubEx.class,"Control Hub");
        PIDCoefficients coefficients = new PIDCoefficients(0.1,0.05,1);

    }


    /**
     * generic intake for Tele Op
     * @param power power to run the intake
     */
    public void intake(double power){
        roller.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        roller.setPower(power);
        upperRoller.setPower(-power);
    }

    /**
     * runs intake for specified number of millis, for Auto
     * @param power power to run the intake
     * @param millis millis intake should run
     */
    public void intake(double power, long millis){
        roller.setPower(power);
        upperRoller.setPower(-power);
        Utils.sleep(millis);
        roller.setPower(1);
    }

    public void setFlywheelVelocity(double angularRate, AngleUnit unit){
        flywheel.setVelocity(angularRate, unit);
    }


    public void shoot(int rings, double angularRate) {
        //Debug.clear();
        flywheel.setVelocity(angularRate, AngleUnit.DEGREES);
        //Thread.sleep(500);
        for (int i = 0; i < rings; i++) {
            flywheel.setVelocity(angularRate, AngleUnit.DEGREES);
            while (Math.abs((flywheel.getVelocity(AngleUnit.DEGREES) - angularRate)) > 6 && AbstractOpMode.currentOpMode().opModeIsActive()){
                //Debug.log(flywheel.getVelocity(AngleUnit.DEGREES));
            }
            //Debug.log(flywheel.getVelocity(AngleUnit.DEGREES));
            indexer.setPosition(1);
            try {
                Thread.sleep(250);
                indexer.setPosition(0.4);
                //flywheel.setVelocity(1600, AngleUnit.DEGREES); //2000
                //Thread.sleep(100); //500
            } catch (InterruptedException e) {
                e.printStackTrace();
            }

        }
        flywheel.setVelocity(0);
        indexer.setPosition(0.4);
    }




    public boolean isNear(double angularRate){
        boolean isNear = (Math.abs(flywheel.getVelocity(AngleUnit.DEGREES) - angularRate) < 10.0);
        //Debug.log(isNear);
        return isNear;
    }





    public void progressBanana(){
        if(indexer.getPosition() == 0.4){
            indexer.setPosition(1);
        }else if(indexer.getPosition() == 1){
            indexer.setPosition(0.4);
        }else{
            indexer.setPosition(1.0);
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
        indexer.setPosition(0.4);
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

    public void setPower(double power) {
        flywheel.setPower(power);
    }

    public void zero() {
        flywheel.setPower(0);
        roller.setPower(0);
        upperRoller.setPower(0);

    }

    /**
     * keeping this code for later, DO NOT DELETE OR CALL
     */

    private void autoOrient(){
        //double distanceAwayX = Constants.GOAL_POSITION.x - Localizer.thisLocalizer().getCurrentPosition().x;
        //double distanceAwayY = Constants.GOAL_POSITION.y - Localizer.thisLocalizer().getCurrentPosition().y;
       // double angleDiff = Math.atan2(distanceAwayY, distanceAwayX);
       // drive.rotate(headingToDirectionRads(angleDiff), 0.4);
        //double distanceAway = Math.sqrt(Math.pow(distanceAwayX, 2) + Math.pow(distanceAwayY, 2));
    }

    public void setFlywheelPower(double power){
        flywheel.setPower(power);
    }

    public double getFlywheelVelocity() {
        return flywheel.getVelocity(AngleUnit.DEGREES);
    }
    public double getFlywheelRadians(){
        return flywheel.getVelocity(AngleUnit.RADIANS);
    }



    //TODO add endgame functions and game specific electronics
    //Wobble Grabber
}
