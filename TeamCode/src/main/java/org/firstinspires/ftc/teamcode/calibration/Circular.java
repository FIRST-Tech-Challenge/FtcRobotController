package org.firstinspires.ftc.teamcode.calibration;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.teamcode.bots.YellowBot;


@TeleOp(name="Calibration Turn", group="Robot15173")
@Disabled
public class Circular extends LinearOpMode {

    private YellowBot bot = new YellowBot();
    ElapsedTime timer = new ElapsedTime();

    private double rightLong = 0;
    private double rightPerDegree = 0;

    private double leftLong = 0;
    private double leftPerDegree = 0;

    private double minRadiusLeft = 0;
    private double minRadiusRight = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        try {
            bot.init(this, hardwareMap, telemetry);
            bot.initGyro();

            telemetry.addData("Status", "Ready for calibration....");
            telemetry.update();

            waitForStart();

            turnLeft();
            timer.reset();
            while(timer.milliseconds() < 3000 && opModeIsActive()){
                telemetry.addData("Calib","Waiting to turn right ...");
                telemetry.update();
            }

            turnRight();

            timer.reset();
            while(timer.milliseconds() < 3000 && opModeIsActive()){
                telemetry.addData("Calib","Waiting for gyro to settle ...");
                telemetry.update();
            }

            BotCalibConfig config = bot.getCalibConfig();
            if (config == null){
                config = new BotCalibConfig();
            }

            config.setMinRadiusLeft(minRadiusLeft);
            config.setMinRadiusRight(minRadiusRight);

            ReadWriteFile.writeFile(bot.getCalibConfigFile(), config.serialize());

            while (opModeIsActive()) {

                telemetry.addData("minRadiusLeft", minRadiusLeft);


                telemetry.addData("minRadiusRight", minRadiusRight);

                telemetry.update();
            }
        }
        catch (Exception ex){
            telemetry.addData("Error", ex.getMessage());
            telemetry.update();
        }
    }

    private void turnLeft(){
        double currentHead = bot.getGyroHeading();
        double desiredHead = currentHead + 90;
        while (bot.getGyroHeading() < desiredHead && opModeIsActive()){
            if (bot.getGyroHeading() < desiredHead/2){
                bot.turnLeft(bot.CALIB_SPEED, true);
            }else{
                bot.turnLeft(bot.CALIB_SPEED/2, true);
            }
            telemetry.addData("Heading", bot.getGyroHeading());
            telemetry.update();
        }

        bot.stop();

        timer.reset();
        while(timer.milliseconds() < 1000 && opModeIsActive()){
            telemetry.addData("Gyroscope","Stabilizing ...");
            telemetry.update();
        }

        double finalHead = bot.getGyroHeading();
        double actualAngle = Math.abs(finalHead - currentHead);

        rightLong = bot.getRightOdometer();

        double dLeft = bot.getLeftOdometer();
        double dCenter = (dLeft + rightLong)/2;
        double dCenterInches = dCenter/bot.COUNTS_PER_INCH_REV;
        double inchPerDegree = dCenterInches/actualAngle;
        double circleLength = inchPerDegree*360;
        minRadiusLeft = circleLength/Math.PI/2;

    }

    private void turnRight(){
        double currentHead = bot.getGyroHeading();
        double desiredHead = currentHead - 90;
        double startLeft = bot.getLeftOdometer();
        double startRight = bot.getRightOdometer();
        while (bot.getGyroHeading() > desiredHead && opModeIsActive()){
            if (bot.getGyroHeading() > desiredHead/2){
                bot.turnRight(bot.CALIB_SPEED, true);
            }else{
                bot.turnRight(bot.CALIB_SPEED/2, true);
            }
            telemetry.addData("Heading", bot.getGyroHeading());
            telemetry.update();
        }

        bot.stop();

        timer.reset();
        while(timer.milliseconds() < 1000 && opModeIsActive()){
            telemetry.addData("Gyroscope","Stabilizing ...");
            telemetry.update();
        }

        double finalHead = bot.getGyroHeading();
        double actualAngle = currentHead - finalHead;

        leftLong = bot.getLeftOdometer() - startLeft;
        double right = bot.getRightOdometer() - startRight;

        double dCenter = (leftLong + right)/2;
        double dCenterInches = dCenter/bot.COUNTS_PER_INCH_REV;
        double inchPerDegree = dCenterInches/actualAngle;
        double circleLength = inchPerDegree*360;
        minRadiusRight = circleLength/Math.PI/2;
    }
}
