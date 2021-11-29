package org.firstinspires.ftc.teamcode.calibration;

import android.graphics.Point;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.teamcode.bots.FrenzyBot;
import org.firstinspires.ftc.teamcode.odometry.IBaseOdometry;
import org.firstinspires.ftc.teamcode.odometry.RobotCoordinatePosition;
import org.firstinspires.ftc.teamcode.odometry.VSlamOdometry;

//import org.openftc.revextensions2.ExpansionHubEx;
//import org.openftc.revextensions2.ExpansionHubMotor;


@TeleOp(name="MasterCalib Cam", group="Robot15173")
//@Disabled
public class MasterCalibCam extends MasterCalib {

    protected void initBot(){
        this.bot = new FrenzyBot();
    }

    protected IBaseOdometry initLocator(){
        this.locator = VSlamOdometry.getInstance(hardwareMap);
        getLocator().init(new Point(startX, startY), lastOrientation);
        Thread positionThread = new Thread(getLocator());
        positionThread.start();
        return this.locator;
    }

    @Override
    protected void calibSpin() {
        IBaseOdometry locator = this.getLocator();
        double currentHead = locator.getOrientation();
        double desiredHead = currentHead + 90;
        double horizontalStart = bot.getHorizontalOdometer();
        double leftStart = bot.getLeftOdometer();
        double rightStart = bot.getRightOdometer();
        while (locator.getOrientation() < desiredHead && opModeIsActive()){
            if (locator.getOrientation() < desiredHead/2){
                bot.spinLeft(desiredSpeed, true);
            }else{
                bot.spinLeft(desiredSpeed/2, true);
            }
            telemetry.addData("Heading", locator.getOrientation());
            telemetry.update();
        }

        bot.stop();

        timer.reset();
        while(timer.milliseconds() < 1000 && opModeIsActive()){
            telemetry.addData("Gyroscope","Stabilizing ...");
            telemetry.update();
        }


        double finalHead = locator.getOrientation();
        double actualAngle = finalHead - currentHead;

        double rightLong = bot.getRightOdometer();

        double leftLong = bot.getLeftOdometer();

        double leftDist = Math.abs(leftLong - leftStart);
        double rightDist = Math.abs(rightLong - rightStart);




        double horizontalPosition = bot.getHorizontalOdometer();
        double horizontalShift = horizontalPosition - horizontalStart;

        horizontalTicksDegreeLeft = Math.abs(horizontalShift/actualAngle);


        //separation
        double ticksPerDegree = (leftDist + rightDist)/actualAngle;
        double circumferance = 90 * ticksPerDegree;
        double circumferanceInches = circumferance / bot.getEncoderCountsPerInch();
        separation = Math.abs(circumferanceInches / Math.PI);
//        separation = 2*90 * ((leftLong - rightLong)/actualAngle)/(Math.PI*bot.getEncoderCountsPerInch());

        BotCalibConfig config = bot.getCalibConfig();
        if (config == null){
            config = new BotCalibConfig();
        }

        config.setWheelBaseSeparation(separation);
        config.setHorizontalTicksDegreeLeft(horizontalTicksDegreeLeft);
        config.setHorizontalTicksDegreeRight(horizontalTicksDegreeRight);
        config.setLeftTicksPerDegree(Math.abs(leftDist/actualAngle));
        config.setRightTicksPerDegree(Math.abs(rightDist/actualAngle));
        ReadWriteFile.writeFile(bot.getCalibConfigFile(), config.serialize());

        timer.reset();
        while(timer.milliseconds() < 1000 && opModeIsActive()){
            telemetry.addData("Spin","Finalizing....");
            telemetry.update();
        }

//        restoreHead();

        telemetry.addData("Spin","Calibration complete");
        telemetry.addData("initial orientation", currentHead);

        telemetry.addData("separation", separation);
        telemetry.addData("rightLong", rightLong);
        telemetry.addData("leftLong", leftLong);
        telemetry.addData("ticksPerDegree", ticksPerDegree);
        telemetry.addData("circumferanceInches", circumferanceInches);
        telemetry.addData("horizontalTicksDegree Left", horizontalTicksDegreeLeft);
        telemetry.addData("horizontalTicksDegree Right", horizontalTicksDegreeRight);
        telemetry.addData("actualAngle Left", actualAngle);
        telemetry.addData("LeftTicksDegree", Math.abs(leftDist/actualAngle));
        telemetry.addData("RightTicksDegree", Math.abs(rightDist/actualAngle));

        telemetry.update();
    }

    @Override
    protected void diagBot(MotorReductionBotCalib calibLeft, MotorReductionBotCalib calibRight){
//        led.none();
        MotorReductionBot mrLeft = calibLeft.getMR();
        MotorReductionBot mrRight = calibRight.getMR();

        double currentHead = getLocator().getOrientation();

        double leftOdo = bot.getLeftOdometer();
        double rightOdo = bot.getRightOdometer();
        bot.diagToCalib(desiredSpeed, 0, desiredX, true, mrLeft, locator);

        timer.reset();
        while(timer.milliseconds() < 1000 && opModeIsActive()){

        }

        double actualHead = getLocator().getOrientation();
        double headChange = Math.abs(actualHead - currentHead);

        double leftDistance = Math.abs(bot.getLeftOdometer() - leftOdo);
        double rightDistance = Math.abs(bot.getRightOdometer() - rightOdo);
        calibLeft.setLeftOdoDistanceActual(leftDistance);
        calibLeft.setRightOdoDistanceActual(rightDistance);
        calibLeft.setHeadChange(headChange);
        calibLeft.process(false);

//        restoreHead();

        timer.reset();
        while(timer.milliseconds() < 1000 && opModeIsActive()){
        }
        currentHead = getLocator().getOrientation();
        leftOdo = bot.getLeftOdometer();
        rightOdo = bot.getRightOdometer();

        bot.diagToCalib(desiredSpeed, 0, desiredX, false, mrRight, locator);

        timer.reset();
        while(timer.milliseconds() < 1000 && opModeIsActive()){

        }

        actualHead = getLocator().getOrientation();
        headChange = Math.abs(actualHead - currentHead);

        calibRight.setHeadChange(headChange);

        leftDistance = Math.abs(bot.getLeftOdometer() - leftOdo);
        rightDistance = Math.abs(bot.getRightOdometer() - rightOdo);

        calibRight.setLeftOdoDistanceActual(leftDistance);
        calibRight.setRightOdoDistanceActual(rightDistance);
        calibRight.process(false);
    }
}
