package org.firstinspires.ftc.teamcode.calibration;

import android.graphics.Point;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import org.firstinspires.ftc.teamcode.bots.BotMoveProfile;
import org.firstinspires.ftc.teamcode.bots.MoveStrategy;
import org.firstinspires.ftc.teamcode.bots.RobotDirection;
import org.firstinspires.ftc.teamcode.bots.RobotMovementStats;
import org.firstinspires.ftc.teamcode.bots.RobotVeer;
import org.firstinspires.ftc.teamcode.bots.YellowBot;
import org.firstinspires.ftc.teamcode.odometry.RobotCoordinatePosition;
import org.firstinspires.ftc.teamcode.skills.Geometry;
import org.firstinspires.ftc.teamcode.skills.Led;
//import org.openftc.revextensions2.ExpansionHubEx;
//import org.openftc.revextensions2.ExpansionHubMotor;

import java.util.concurrent.TimeUnit;


@TeleOp(name="MasterCalib", group="Robot15173")
public class MasterCalib extends LinearOpMode {

    private YellowBot bot = new YellowBot();
    ElapsedTime timer = new ElapsedTime();


    private static double CALIB_SPEED = 0.5;
    private static double CALIB_SPEED_HIGH = 0.9;
    private static double CALIB_SPEED_LOW = 0.2;

    private static double MARGIN_ERROR_DEGREES = 2;

    private static double MARGIN_ERROR_RADIUS = 4;


    private double separation = 0;
    private double horizontalTicksDegreeLeft = 0;
    private double horizontalTicksDegreeRight = 0;


    Deadline gamepadRateLimit;
    private final static int GAMEPAD_LOCKOUT = 500;

    private static final int[] modes = new int[]{0, 1, 2, 3, 4, 5, 6, 7, 8, 9};
    private static final String[] modeNames = new String[]{"Straight", "Curve", "Break", "Spin", "Strafe", "Diag", "Spin Calib", "Calib Full", "Save", "Save Amps"};

    private int selectedMode = 0;

    private boolean speedSettingMode = false;
    private boolean MRSettingMode = false;
    private boolean MRSettingModeBack = false;
    private boolean XSettingMode = false;
    private boolean YSettingMode = false;
    private boolean XFromSettingMode = true;
    private boolean YFromSettingMode = false;
    private boolean strafeModeLeft = false;
    private boolean strafeModeRight = false;
    private boolean diagModeLeft = false;
    private boolean diagModeRight = false;

    private boolean saveMode = false;


    private boolean bpSettingMode = false;
    private boolean routeSettingMode  = false;


    private boolean tenIncrement = false;

    private double breakPointOverride = 0;
    private double lastOrientation = 0;

    private int desiredX = 30;
    private int desiredY = 75;
    int startX = 30;
    int startY = 25;
    private static int DIAG = 45;
    private double desiredSpeed = CALIB_SPEED;

    MotorReductionBotCalib templateMRForward = new MotorReductionBotCalib();
    MotorReductionBotCalib templateMRBack = new MotorReductionBotCalib();

    MotorReductionBotCalib templateStrafeLeft = new MotorReductionBotCalib();
    MotorReductionBotCalib templateStrafeRight = new MotorReductionBotCalib();

    MotorReductionBotCalib templateDiagLeft = new MotorReductionBotCalib();
    MotorReductionBotCalib templateDiagRight = new MotorReductionBotCalib();

    private Led led = null;


    @Override
    public void runOpMode() throws InterruptedException {
        try {
            bot.init(this, hardwareMap, telemetry);
            bot.initGyro();
//            bot.initCalibData();

            BotCalibConfig config = bot.getCalibConfig();
            if (config != null){
                this.templateMRForward = new MotorReductionBotCalib(config.getMoveMRForward());
                this.templateMRBack = new MotorReductionBotCalib(config.getMoveMRBack());

                this.templateStrafeLeft = new MotorReductionBotCalib(config.getStrafeLeftReduction());
                this.templateStrafeRight = new MotorReductionBotCalib(config.getStrafeRightReduction());

                this.templateDiagLeft = new MotorReductionBotCalib(config.getDiagMRLeft());
                this.templateDiagRight = new MotorReductionBotCalib(config.getDiagMRRight());
            }
            this.templateMRBack.setDirection(RobotDirection.Backward);

            this.templateStrafeLeft.setDirection(RobotDirection.Left);
            this.templateStrafeRight.setDirection(RobotDirection.Right);

            this.templateDiagLeft.setDirection(RobotDirection.Left);
            this.templateDiagRight.setDirection(RobotDirection.Right);


            this.led = bot.getLights();

            gamepadRateLimit = new Deadline(GAMEPAD_LOCKOUT, TimeUnit.MILLISECONDS);


//            try {
//                ExpansionHubEx expansionHubRight = hardwareMap.get(ExpansionHubEx.class, "Right Hub 1");
//                if (expansionHubRight != null) {
//                    telemetry.addData("Right Hub", "Stats:");
//                    telemetry.addData("Battery monitor, volts", expansionHubRight.read12vMonitor(ExpansionHubEx.VoltageUnits.VOLTS));
//                    telemetry.addData("Firmware v", expansionHubRight.getFirmwareVersion());
//                    telemetry.addData("Hardware rev", expansionHubRight.getHardwareRevision());
//                    telemetry.addData("Total current", expansionHubRight.getTotalModuleCurrentDraw(ExpansionHubEx.CurrentDrawUnits.AMPS));
//                    ExpansionHubMotor backRight = (ExpansionHubMotor) hardwareMap.dcMotor.get("backRight");
//                    ExpansionHubMotor frontRight = (ExpansionHubMotor) hardwareMap.dcMotor.get("frontRight");
//
//                    telemetry.addData("RF current", frontRight.getCurrentDraw(ExpansionHubEx.CurrentDrawUnits.AMPS));
//                    telemetry.addData("RB current", backRight.getCurrentDraw(ExpansionHubEx.CurrentDrawUnits.AMPS));
//
//                }
//
//                ExpansionHubEx expansionHubLeft = hardwareMap.get(ExpansionHubEx.class, "Left Hub 2");
//                if (expansionHubLeft != null) {
//                    telemetry.addData("Left Hub", "Stats:");
//                    telemetry.addData("Battery monitor, volts", expansionHubLeft.read12vMonitor(ExpansionHubEx.VoltageUnits.VOLTS));
//                    telemetry.addData("Firmware v", expansionHubLeft.getFirmwareVersion());
//                    telemetry.addData("Hardware rev", expansionHubLeft.getHardwareRevision());
//                    telemetry.addData("Total current", expansionHubLeft.getTotalModuleCurrentDraw(ExpansionHubEx.CurrentDrawUnits.AMPS));
//                    ExpansionHubMotor backLeft = (ExpansionHubMotor) hardwareMap.dcMotor.get("backLeft");
//                    ExpansionHubMotor frontLeft = (ExpansionHubMotor) hardwareMap.dcMotor.get("frontLeft");
//
//                    telemetry.addData("LF current", frontLeft.getCurrentDraw(ExpansionHubEx.CurrentDrawUnits.AMPS));
//                    telemetry.addData("LB current", backLeft.getCurrentDraw(ExpansionHubEx.CurrentDrawUnits.AMPS));
//                }
//            }
//            catch (Exception ex){
//                telemetry.addData("Error", String.format("Issues whe getting extended info....%s"), ex.getMessage());
//                telemetry.update();
//            }

            telemetry.addData("Master Cali", "Ready to calibrate....");
            telemetry.update();

            waitForStart();
            showStatus();

            while (opModeIsActive()) {
                processCommands();
            }
        }
        catch (Exception ex){
            telemetry.addData("Error", ex.getMessage());
            telemetry.update();
            sleep(5000);
        }
    }

    private void changeMoveModes(){
        if (!MRSettingMode && !MRSettingModeBack){
            MRSettingMode = true;
        }
        else if (MRSettingMode){
            MRSettingMode = false;
            MRSettingModeBack = true;
        }
        else if (MRSettingModeBack){
            MRSettingMode = false;
            MRSettingModeBack = false;
        }
    }

    private void changeStrafeModes(){
        if (!strafeModeLeft && !strafeModeRight){
            strafeModeLeft = true;
        }
        else if (strafeModeLeft){
            strafeModeLeft = false;
            strafeModeRight = true;
        }
        else if (strafeModeRight){
            strafeModeLeft = false;
            strafeModeRight = false;
        }
    }

    private void changeDiagModes(){
        if (!diagModeLeft && !diagModeRight){
            diagModeLeft = true;
        }
        else if (diagModeLeft){
            diagModeLeft = false;
            diagModeRight = true;
        }
        else if (diagModeRight){
            diagModeLeft = false;
            diagModeRight = false;
        }
    }


    private void processCommands(){
        if (!gamepadRateLimit.hasExpired()) {
            return;
        }


        if(gamepad1.a){
            speedSettingMode = !speedSettingMode;
            gamepadRateLimit.reset();
            showStatus();
        }

        if (gamepad1.b){
            bpSettingMode = !bpSettingMode;
            gamepadRateLimit.reset();
            showStatus();
        }

        if (gamepad1.x){
            routeSettingMode = !routeSettingMode;
            gamepadRateLimit.reset();
            showStatus();
        }

        if (gamepad1.y){
            routeSettingMode = !routeSettingMode;
            gamepadRateLimit.reset();
            showStatus();
        }


        if (gamepad1.left_bumper){
            tenIncrement = !tenIncrement;
            gamepadRateLimit.reset();
        }

        if (gamepad1.right_bumper){
            //swap to and from coordinates
            if (routeSettingMode){
                int tempX = desiredX;
                int tempY = desiredY;
                desiredX = startX;
                desiredY = startY;
                startX = tempX;
                startY = tempY;
                gamepadRateLimit.reset();
                showStatus();
            }
        }

        if (gamepad1.back){
            switch (selectedMode) {
                case 0:
                case 1:
                case 2:
                    changeMoveModes();
                    break;
                case 4:
                    changeStrafeModes();
                    break;
                case 5:
                    changeDiagModes();
                    break;
            }

            gamepadRateLimit.reset();
            showStatus();
        }


        if (gamepad1.dpad_down){
            if (speedSettingMode){
                if (desiredSpeed > 0){
                    desiredSpeed = desiredSpeed - 0.1;
                    desiredSpeed = Math.round(desiredSpeed*10)/10.0;
                }
            }
            else if (bpSettingMode){
                if (breakPointOverride > 0){
                    breakPointOverride = breakPointOverride - 1;
                }
            }
            else if (routeSettingMode){
                if (XFromSettingMode){
                    startX = startX - 5;
                }
                else if (XSettingMode) {
                    desiredX = desiredX - 5;
                }
                if (YFromSettingMode){
                    startY = startY - 5;
                }
                else if (YSettingMode) {
                    desiredY = desiredY - 5;
                }
            }
            else if (MRSettingMode){
                templateMRForward.decrementSelectedMR(tenIncrement);
            }
            else if (MRSettingModeBack){
                templateMRBack.decrementSelectedMR(tenIncrement);
            }
            else if(strafeModeLeft){
                templateStrafeLeft.decrementSelectedMR(tenIncrement);
            }
            else if(strafeModeRight){
                templateStrafeRight.decrementSelectedMR(tenIncrement);
            }
            else if(diagModeLeft){
                templateDiagLeft.decrementSelectedMR(tenIncrement);
            }
            else if(diagModeRight){
                templateDiagRight.decrementSelectedMR(tenIncrement);
            }
            else {
                if (selectedMode < modes.length) {
                    selectedMode++;
                    saveMode = selectedMode == 8;
                }
            }
            gamepadRateLimit.reset();
            showStatus();
        }

        if (gamepad1.dpad_up){
            if (speedSettingMode){
                if (desiredSpeed < 1){
                    desiredSpeed = desiredSpeed + 0.1;
                    desiredSpeed = Math.round(desiredSpeed*10)/10.0;
                }
            }
            else if (bpSettingMode){
                breakPointOverride = breakPointOverride + 1;
            }
            else if (routeSettingMode){
                if (XFromSettingMode){
                    startX = startX + 5;
                }
                else if (XSettingMode) {
                    desiredX = desiredX + 5;
                }
                if (YFromSettingMode){
                    startY = startY + 5;
                }
                else if (YSettingMode) {
                    desiredY = desiredY + 5;
                }
            }
            else if (MRSettingMode){
                templateMRForward.inrementSelectedMR(tenIncrement);
            }
            else if (MRSettingModeBack){
                templateMRBack.inrementSelectedMR(tenIncrement);
            }
            else if(strafeModeLeft){
                templateStrafeLeft.inrementSelectedMR(tenIncrement);
            }
            else if(strafeModeRight){
                templateStrafeRight.inrementSelectedMR(tenIncrement);
            }
            else if(diagModeLeft){
                templateDiagLeft.inrementSelectedMR(tenIncrement);
            }
            else if(diagModeRight){
                templateDiagRight.inrementSelectedMR(tenIncrement);
            }
            else {
                if (selectedMode > 0) {
                    selectedMode--;
                    saveMode = selectedMode == 8;
                }
            }
            gamepadRateLimit.reset();
            showStatus();
        }
        if (gamepad1.dpad_left){
            if (MRSettingMode){
                templateMRForward.selectPrev();
                gamepadRateLimit.reset();
                showStatus();
            }
            else if(MRSettingModeBack){
                templateMRBack.selectPrev();
                gamepadRateLimit.reset();
                showStatus();
            }
            else if (strafeModeLeft){
                templateStrafeLeft.selectPrev();
                gamepadRateLimit.reset();
                showStatus();
            }
            else if (strafeModeRight){
                templateStrafeRight.selectPrev();
                gamepadRateLimit.reset();
                showStatus();
            }
            else if (diagModeLeft){
                templateDiagLeft.selectPrev();
                gamepadRateLimit.reset();
                showStatus();
            }
            else if (diagModeRight){
                templateDiagRight.selectPrev();
                gamepadRateLimit.reset();
                showStatus();
            }
            else if (routeSettingMode){
                if (XFromSettingMode){
                    XFromSettingMode = false;
                    YFromSettingMode = true;
                }
                else if (YFromSettingMode){
                    YFromSettingMode = false;
                    XSettingMode = true;
                }
                else if (XSettingMode){
                    XSettingMode = false;
                    YSettingMode = true;
                }
                else if (YSettingMode){
                    YSettingMode = false;
                    XFromSettingMode = true;
                }
                gamepadRateLimit.reset();
                showStatus();
            }

        }

        if (gamepad1.dpad_right){
            if (MRSettingMode){
                templateMRForward.selectNext();
                gamepadRateLimit.reset();
                showStatus();
            }
            else if(MRSettingModeBack){
                templateMRBack.selectNext();
                gamepadRateLimit.reset();
                showStatus();
            }
            else if (strafeModeLeft){
                templateStrafeLeft.selectNext();
                gamepadRateLimit.reset();
                showStatus();
            }
            else if (strafeModeRight){
                templateStrafeRight.selectNext();
                gamepadRateLimit.reset();
                showStatus();
            }
            else if (diagModeLeft){
                templateDiagLeft.selectNext();
                gamepadRateLimit.reset();
                showStatus();
            }
            else if (diagModeRight){
                templateDiagRight.selectNext();
                gamepadRateLimit.reset();
                showStatus();
            }
            else if (routeSettingMode){
                if (XFromSettingMode){
                    XFromSettingMode = false;
                    XSettingMode = true;
                }
                else if (XSettingMode){
                    XSettingMode = false;
                    YSettingMode = true;
                }
                else if (YSettingMode){
                    YSettingMode = false;
                    YFromSettingMode = true;
                }
                else if (YFromSettingMode){
                    YFromSettingMode = false;
                    XFromSettingMode = true;
                }
                gamepadRateLimit.reset();
                showStatus();
            }
        }

        if (gamepad1.start){

            switch (selectedMode) {
                case 0:
                    calibMove();
                    break;
                case 1:
                    calibCurve();
                    break;
                case 2:
                    calibBreak();
                    break;
                case 3:
                    calibSpinPrecision();
                    break;
                case 4:
                    calibStrafe();
                    break;
                case 5:
                    calibDiag();
                    break;
                case 6:
                    calibSpin();
                    break;
                case 7:
                    calibAll();
                    break;
                case 8:
                    saveCurrentConfig();
                    break;
                case 9:
                    saveMoveConfigFromAmps();
                    break;

            }
        }
    }

    private void showRoute(){
        String fromX = XFromSettingMode ? "*" : " ";
        String toX = XSettingMode ? "*" : " ";
        String fromY = YFromSettingMode ? "*" : " ";
        String toY = YSettingMode ? "*" : " ";

        telemetry.addData("From", "%d%s : %d%s", startX, fromX, startY, fromY);
        telemetry.addData("To", "%d%s : %d%s", desiredX, toX, desiredY, toY);
    }

    private void showStatus(){

        if (speedSettingMode) {
            telemetry.addData("Speed", desiredSpeed);
        }
        else if (routeSettingMode){
            showRoute();
        }
        else if (bpSettingMode){
            telemetry.addData("Break Point", breakPointOverride);
        }
        else if (MRSettingMode){
            showMotorReductionCalib(templateMRForward);
        }
        else if (MRSettingModeBack){
            showMotorReductionCalib(templateMRBack);
        }
        else if (strafeModeLeft){
            showMotorReductionCalib(templateStrafeLeft);
        }
        else if (strafeModeRight){
            showMotorReductionCalib(templateStrafeRight);
        }
        else if (diagModeLeft){
            showMotorReductionCalib(templateDiagLeft);
        }
        else if (diagModeRight){
            showMotorReductionCalib(templateDiagRight);
        }
        else {
            for (int i = 0; i < modes.length; i++) {
                telemetry.addData(modeNames[i], i == selectedMode);
            }
        }

        telemetry.update();
    }


    private void calibSpinPrecision(){
        RobotCoordinatePosition locator = null;
        try {
            //tracker
            locator = new RobotCoordinatePosition(bot, new Point(startX, startY), lastOrientation, RobotCoordinatePosition.THREAD_INTERVAL);
            locator.reverseHorEncoder();
            Thread positionThread = new Thread(locator);
            positionThread.start();
            int selectedIndex = MotorReductionBot.POWER_SAMPLES.length - 1;
            while (selectedIndex >= 0) {
                double power = MotorReductionBot.POWER_SAMPLES[selectedIndex];
                calibSpinPrecision(power, true, locator);

                timer.reset();
                while (timer.milliseconds() < 1000 && opModeIsActive()) {

                }
                calibSpinPrecision(power, false, locator);
                selectedIndex--;

            }
            ReadWriteFile.writeFile(bot.getCalibConfigFile(), bot.getCalibConfig().serialize());

            //try with calibration
            telemetry.addData("Testing", "");
            testSpinPrecision(desiredSpeed, true, locator);
            testSpinPrecision(desiredSpeed, false, locator);
            telemetry.update();
        }
        catch (Exception ex){
            telemetry.addData("Error", ex.getMessage());
            telemetry.update();
        }
        finally {
            if(locator != null){
                locator.stop();
            }
        }
    }

    private void calibSpinPrecision(double speed, boolean left, RobotCoordinatePosition locator){
        BotCalibConfig config = bot.getCalibConfig();
        try {

            double degrees = 90;
            if (left == false){
                degrees = -degrees;
            }

            double desiredChange = Math.abs(degrees);
            double startHead = locator.getOrientation();


            bot.spinCalib(degrees, speed, locator);

            timer.reset();
            while(timer.milliseconds() < 1000 && opModeIsActive()){

            }

            double endHead = locator.getOrientation();

            double actualChange = Math.abs(endHead - startHead);
            double reduction = 1;

            if (actualChange > desiredChange){
                reduction = 1 - ((actualChange - desiredChange)/desiredChange);
            }

            if (left) {
                config.getSpinLeftConfig().setBreakPoint(reduction, speed);
            }
            else{
                config.getSpinRightConfig().setBreakPoint(reduction, speed);
            }
        }
        catch (Exception ex){
            telemetry.addData("Error", ex.getMessage());
            telemetry.update();
        }
    }

    private void testSpinPrecision(double speed, boolean left, RobotCoordinatePosition locator){
        BotCalibConfig config = bot.getCalibConfig();
        try {

            double degrees = 90;
            if (left == false){
                degrees = -degrees;
            }

            double desiredChange = Math.abs(degrees);
            double startHead = locator.getOrientation();

            BotMoveProfile profile = new BotMoveProfile(MoveStrategy.Spin);
            profile.setAngleChange(degrees);
            profile.setTopSpeed(speed);

            bot.spin(profile, locator);

            timer.reset();
            while(timer.milliseconds() < 1000 && opModeIsActive()){

            }

            double endHead = locator.getOrientation();

            double actualChange = Math.abs(endHead - startHead);

            if (Math.abs(actualChange) <= MARGIN_ERROR_DEGREES){
                led.OK();
            }
            else{
                led.needAdjustment();
            }


            telemetry.addData("left", left);
            telemetry.addData("desired", desiredChange);
            telemetry.addData("actual", actualChange);
            timer.reset();
            while(timer.milliseconds() < 2000 && opModeIsActive()){

            }

            led.none();
        }
        catch (Exception ex){
            telemetry.addData("Error", ex.getMessage());
            telemetry.update();
        }

    }

    private void calibCurve(){
        RobotCoordinatePosition locator = null;
        try {
            //tracker
            locator = new RobotCoordinatePosition(bot, new Point(startX, startY), lastOrientation,RobotCoordinatePosition.THREAD_INTERVAL);
            locator.reverseHorEncoder();
            Thread positionThread = new Thread(locator);
            positionThread.start();

            BotMoveProfile profile = BotMoveProfile.bestRoute(bot, startX, startY, new Point(desiredX, desiredY), RobotDirection.Optimal, desiredSpeed, MoveStrategy.Curve, BotMoveProfile.DEFAULT_HEADING, locator);
            profile.setStart(new Point(startX, startY));
            profile.setDestination(new Point(desiredX, desiredY));
            //override MR for calibration purposes
            if (profile.getDirection() == RobotDirection.Forward){
                profile.setMotorReduction(templateMRForward);
            }
            else{
                profile.setMotorReduction(templateMRBack);
            }

            bot.curveTo(profile, locator);

            timer.reset();
            while(timer.milliseconds() < 2000 && opModeIsActive()){

            }


            profile.setActual(new Point((int)Math.round(locator.getXInches()), (int)Math.round(locator.getYInches())));

            lastOrientation = locator.getOrientation();



            Point newStart = new Point((int)Math.round(locator.getXInches()), (int) Math.round(locator.getYInches()));

            BotMoveProfile whatIfBack = BotMoveProfile.bestRoute(bot, locator.getXInches(), locator.getYInches(), new Point(startX, startY), RobotDirection.Optimal, desiredSpeed, MoveStrategy.Curve, BotMoveProfile.DEFAULT_HEADING, locator);
            bot.curveTo(whatIfBack, locator);
            whatIfBack.setStart(newStart);
            whatIfBack.setDestination(new Point(startX, startY));
            whatIfBack.setActual(new Point((int)Math.round(locator.getXInches()), (int)Math.round(locator.getYInches())));

            telemetry.addData("Profile Original", profile.toString());
            showMotorReduction(profile.getMotorReduction());
            telemetry.addData("Profile Back", whatIfBack.toString());


            telemetry.update();

        }
        catch (Exception ex){
            telemetry.addData("Error", ex.getMessage());
            telemetry.update();
        }
        finally {
            if (locator != null){
                locator.stop();
            }
        }
    }

    private void calibMove(){
        RobotCoordinatePosition locator = null;
        try {
            locator = new RobotCoordinatePosition(bot, new Point(startX, startY), lastOrientation,RobotCoordinatePosition.THREAD_INTERVAL);
            locator.reverseHorEncoder();
            Thread positionThread = new Thread(locator);
            positionThread.start();
            moveBot(templateMRForward, templateMRBack, locator);
//            restoreHead();
            led.none();
            showMotorReductionCalib(templateMRForward);
            showMotorReductionCalib(templateMRBack);
            telemetry.addData("Question", "Save MR from Amps? Yes - Press Start. No - Any key");
            telemetry.update();

        }
        finally {
            if (locator != null){
                locator.stop();
            }
        }
    }

    private void moveBot(MotorReductionBotCalib calibF, MotorReductionBotCalib calibB, RobotCoordinatePosition locator){
        led.none();
        MotorReductionBot mrForward = calibF.getMR();
        MotorReductionBot mrBack = calibB.getMR();

        double currentHead = bot.getGyroHeading();

        double leftOdo = bot.getLeftOdometer();
        double rightOdo = bot.getRightOdometer();
        calibF.setLeftOdoDistance(desiredX*bot.COUNTS_PER_INCH_REV);
        calibF.setRightOdoDistance(desiredX*bot.COUNTS_PER_INCH_REV);
        double bF = calibF.getBreakPoint(desiredSpeed);
        if (breakPointOverride > 0){
            bF = breakPointOverride * bot.COUNTS_PER_INCH_REV;
        }
        int distance = Math.abs(desiredY - startY);
        RobotMovementStats statsF = bot.moveToCalib(desiredSpeed, desiredSpeed, distance, mrForward, bF, led);
        calibF.setStats(statsF);

        timer.reset();
        while(timer.milliseconds() < 1000 && opModeIsActive()){

        }
//        restoreHead();

        double actualHead = bot.getGyroHeading();
        double headChange = Math.abs(actualHead - currentHead);

        double leftDistance = Math.abs(bot.getLeftOdometer() - leftOdo);
        double rightDistance = Math.abs(bot.getRightOdometer() - rightOdo);
        calibF.setLeftOdoDistanceActual(leftDistance);
        calibF.setRightOdoDistanceActual(rightDistance);
        calibF.setHeadChange(headChange);
        calibF.process(false);
        double distanceFromTarget = Geometry.getDistance(locator.getXInches(), locator.getYInches(), startX, desiredY);
        calibF.setDistanceFromTarget(distanceFromTarget );

        if (distanceFromTarget > MARGIN_ERROR_RADIUS){
            led.needAdjustment();
        }
        else{
            saveConfigMoveForward(templateMRForward);
            led.OK();
        }


        timer.reset();
        while(timer.milliseconds() < 1000 && opModeIsActive()){
        }
        currentHead = bot.getGyroHeading();
        leftOdo = bot.getLeftOdometer();
        rightOdo = bot.getRightOdometer();
        calibB.setLeftOdoDistance(desiredX*bot.COUNTS_PER_INCH_REV);
        calibB.setRightOdoDistance(desiredX*bot.COUNTS_PER_INCH_REV);


        double bB = calibB.getBreakPoint(desiredSpeed);
        if (breakPointOverride > 0){
            bF = breakPointOverride * bot.COUNTS_PER_INCH_REV;
        }

        telemetry.addData("Forw Location", "x:%.2f  y: %.2f ", locator.getXInches(), locator.getYInches());


        RobotMovementStats statsB =  bot.moveToCalib(desiredSpeed, desiredSpeed, -distance, mrBack, bB, led);
        calibB.setStats(statsB);
        telemetry.addData("Actual BP Forw", bF);
        telemetry.addData("Actual BP Back", bB);

        timer.reset();
        while(timer.milliseconds() < 1000 && opModeIsActive()){

        }

        actualHead = bot.getGyroHeading();
        headChange = Math.abs(actualHead - currentHead);

        calibB.setHeadChange(headChange);
        double distanceFromTargetBack = Geometry.getDistance(locator.getXInches(), locator.getYInches(), startX, startY);
        calibB.setDistanceFromTarget(distanceFromTargetBack);

        telemetry.addData("Back Location", "x:%.2f  y: %.2f ", locator.getXInches(), locator.getYInches());

        if (distanceFromTargetBack > MARGIN_ERROR_RADIUS){
            led.needAdjustment();
        }
        else{
            led.OK();
            saveConfigMoveForward(templateMRBack);
        }

        leftDistance = Math.abs(bot.getLeftOdometer() - leftOdo);
        rightDistance = Math.abs(bot.getRightOdometer() - rightOdo);

        calibB.setLeftOdoDistanceActual(leftDistance);
        calibB.setRightOdoDistanceActual(rightDistance);
        calibB.process(false);
    }



    private void calibBreak(){
        BotStatsConfig statsConfig = BotStatsConfig.loadConfig(telemetry);
        int selectedIndex = 0;
        while(selectedIndex < MotorReductionBot.POWER_SAMPLES.length){
            double power = MotorReductionBot.POWER_SAMPLES[selectedIndex];
            RobotMovementStats statsF =  bot.moveToCalib(power, power, desiredX, templateMRForward, 0, led);
            templateMRForward.setBreakPoint(statsF.getSlowDownDistanceRaw(), power);
            statsConfig.setStatsForward(power, statsF);

            timer.reset();
            while (timer.milliseconds() < 1000 && opModeIsActive()) {

            }
            RobotMovementStats statsB =  bot.moveToCalib(power, power, -desiredX, templateMRBack, 0, led);
            templateMRBack.setBreakPoint(statsB.getSlowDownDistanceRaw(), power);
            statsConfig.setStatsBack(power, statsB);
            selectedIndex++;

        }
        saveConfigBreak(templateMRForward, templateMRBack);
        statsConfig.saveConfig(telemetry);
    }


    private void calibTurn(){
        double minRadiusLeft = turnLeft();
        double minRadiusRight = turnRight();
        BotCalibConfig config = bot.getCalibConfig();
        if (config == null){
            config = new BotCalibConfig();
        }

        config.setMinRadiusLeft(minRadiusLeft);
        config.setMinRadiusRight(minRadiusRight);

        ReadWriteFile.writeFile(bot.getCalibConfigFile(), config.serialize());

        telemetry.addData("min radius left turn ", config.getMinRadiusLeft());
        telemetry.addData("min radius right turn ", config.getMinRadiusRight());
        telemetry.update();
    }

    private double turnLeft(){
        double minRadiusLeft = 0;
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

        double rightLong = bot.getRightOdometer();

        double dLeft = bot.getLeftOdometer();
        double dCenter = (dLeft + rightLong)/2;
        double dCenterInches = dCenter/bot.COUNTS_PER_INCH_REV;
        double inchPerDegree = dCenterInches/actualAngle;
        double circleLength = inchPerDegree*360;
        minRadiusLeft = circleLength/Math.PI/2;
        return minRadiusLeft;
    }

    private double turnRight(){
        double minRadiusRight = 0;
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

        double leftLong = bot.getLeftOdometer() - startLeft;
        double right = bot.getRightOdometer() - startRight;

        double dCenter = (leftLong + right)/2;
        double dCenterInches = dCenter/bot.COUNTS_PER_INCH_REV;
        double inchPerDegree = dCenterInches/actualAngle;
        double circleLength = inchPerDegree*360;
        minRadiusRight = circleLength/Math.PI/2;
        return minRadiusRight;
    }

    private void calibAll(){
        calibSpin();
        timer.reset();
        while(timer.milliseconds() < 1000 && opModeIsActive()){
        }

//        restoreHead();

        while(timer.milliseconds() < 1000 && opModeIsActive()){
        }

        calibTurn();

//        restoreHead();

        while(timer.milliseconds() < 1000 && opModeIsActive()){
        }

        calibBreak();

        BotCalibConfig config = bot.getCalibConfig();
        if (config == null){
            telemetry.addData("SOS", "Something is totally wrong. Bot config is missing");
            telemetry.update();
        }
        else {


            telemetry.addData("separation", config.getWheelBaseSeparation());
            telemetry.addData("horizontalTicksDegree Left", config.getHorizontalTicksDegreeLeft());
            telemetry.addData("horizontalTicksDegree Right", config.getHorizontalTicksDegreeRight());
            telemetry.addData("separation", config.getWheelBaseSeparation());
            telemetry.addData("min radius left turn ", config.getMinRadiusLeft());
            telemetry.addData("min radius right turn ", config.getMinRadiusRight());
            telemetry.update();
        }

    }

    private void calibSpin(){
        double currentHead = bot.getGyroHeading();
        double desiredHead = currentHead + 90;
        double horizontalStart = bot.getHorizontalOdometer();
        double leftStart = bot.getLeftOdometer();
        double rightStart = bot.getRightOdometer();
        while (bot.getGyroHeading() < desiredHead && opModeIsActive()){
            if (bot.getGyroHeading() < desiredHead/2){
                bot.spinLeft(desiredSpeed, true);
            }else{
                bot.spinLeft(desiredSpeed/2, true);
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
        double actualAngle = finalHead - currentHead;

        double rightLong = bot.getRightOdometer();

        double leftLong = bot.getLeftOdometer();

        double leftDist = Math.abs(leftLong - leftStart);
        double rightDist = Math.abs(rightLong - rightStart);



        double horizontalPosition = bot.getHorizontalOdometer();
        double horizontalShift = horizontalPosition - horizontalStart;

        horizontalTicksDegreeLeft = Math.abs(horizontalShift/actualAngle);

        timer.reset();
        while(timer.milliseconds() < 1000 && opModeIsActive()){
            telemetry.addData("Spin","Will go to right now");
            telemetry.update();
        }

        spinRight();



        //separation
        double ticksPerDegree = (leftLong - rightLong)/actualAngle;
        double circumferance = 180 * ticksPerDegree;
        double circumferanceInches = circumferance / bot.COUNTS_PER_INCH_REV;
        separation = Math.abs(circumferanceInches / Math.PI);
//        separation = 2*90 * ((leftLong - rightLong)/actualAngle)/(Math.PI*bot.COUNTS_PER_INCH_REV);

        BotCalibConfig config = bot.getCalibConfig();
        if (config == null){
            config = new BotCalibConfig();
        }

        config.setWheelBaseSeparation(separation);
        config.setHorizontalTicksDegreeLeft(horizontalTicksDegreeLeft);
        config.setHorizontalTicksDegreeRight(horizontalTicksDegreeRight);
        ReadWriteFile.writeFile(bot.getCalibConfigFile(), config.serialize());

        timer.reset();
        while(timer.milliseconds() < 1000 && opModeIsActive()){
            telemetry.addData("Spin","Finalizing....");
            telemetry.update();
        }

//        restoreHead();

        telemetry.addData("Spin","Calibration complete");

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

    private void spinRight(){
        double currentHead = bot.getGyroHeading();
        double desiredHead = currentHead - 90;
        double horizontalStart = bot.getHorizontalOdometer();
        while (bot.getGyroHeading() > desiredHead && opModeIsActive()){
            if (bot.getGyroHeading() > currentHead/2){
                bot.spinRight(desiredSpeed, true);
            }else{
                bot.spinRight(desiredSpeed/2, true);
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
        double actualAngle = finalHead - currentHead;


        double horizontalPosition = bot.getHorizontalOdometer();
        double horizontalShift = horizontalPosition - horizontalStart;

        horizontalTicksDegreeRight = Math.abs(horizontalShift/actualAngle);
    }

    private void calibStrafe(){
        strafeBot(templateStrafeLeft, templateStrafeRight);
//        restoreHead();
        led.none();
//        saveConfigStrafe(templateStrafeLeft, templateStrafeRight);
        showMotorReductionCalib(templateStrafeLeft);
        showMotorReductionCalib(templateStrafeRight);
        telemetry.update();
    }

    private void strafeBot(MotorReductionBotCalib calibLeft, MotorReductionBotCalib calibRight){
        led.none();
        MotorReductionBot mrLeft = calibLeft.getMR();
        MotorReductionBot mrRight = calibRight.getMR();

        double currentHead = bot.getGyroHeading();

        double leftOdo = bot.getLeftOdometer();
        double rightOdo = bot.getRightOdometer();
        bot.strafeToCalib(desiredSpeed, desiredX, true, mrLeft);

        timer.reset();
        while(timer.milliseconds() < 1000 && opModeIsActive()){

        }

        double actualHead = bot.getGyroHeading();
        double headChange = Math.abs(actualHead - currentHead);

        double leftDistance = Math.abs(bot.getLeftOdometer() - leftOdo);
        double rightDistance = Math.abs(bot.getRightOdometer() - rightOdo);
        calibLeft.setLeftOdoDistanceActual(leftDistance);
        calibLeft.setRightOdoDistanceActual(rightDistance);
        calibLeft.setHeadChange(headChange);
        calibLeft.process(true);

        if (Math.abs(headChange) > MARGIN_ERROR_DEGREES){
            led.needAdjustment();
        }
        else{
            led.OK();
        }

//        restoreHead();

        timer.reset();
        while(timer.milliseconds() < 1000 && opModeIsActive()){
        }
        currentHead = bot.getGyroHeading();
        leftOdo = bot.getLeftOdometer();
        rightOdo = bot.getRightOdometer();

        bot.strafeToCalib(desiredSpeed, desiredX, false, mrRight);

        timer.reset();
        while(timer.milliseconds() < 1000 && opModeIsActive()){

        }

        actualHead = bot.getGyroHeading();
        headChange = Math.abs(actualHead - currentHead);

        calibRight.setHeadChange(headChange);

        if (Math.abs(headChange) > MARGIN_ERROR_DEGREES){
            led.needAdjustment();
        }
        else{
            led.OK();
        }

        leftDistance = Math.abs(bot.getLeftOdometer() - leftOdo);
        rightDistance = Math.abs(bot.getRightOdometer() - rightOdo);

        calibRight.setLeftOdoDistanceActual(leftDistance);
        calibRight.setRightOdoDistanceActual(rightDistance);
        calibRight.process(true);
    }

//    private void calibStrafeMulti(){
//        double headChange = 0;
//
//        boolean stop = false;
//        while(!stop) {
//            headChange = strafe();
//            if (currentMRRight.isCalibComplete() && currentMRLeft.isCalibComplete()){
//                break;
//            }
//        }
//        telemetry.addData("Strafe", "Calibration complete");
//        telemetry.addData("Last turnDegrees", headChange);
//        telemetry.addData("Left Reduction", leftReductionStrafe.toString());
//        telemetry.addData("strafeRightReduction", rightReductionStrafe.toString());
//        telemetry.update();
//        saveConfigStrafe();
//    }

//    private double strafe(){
//        led.none();
//
//        double headChange = 0;
//        double currentHead = bot.getGyroHeading();
//
//        double startLeft = bot.getLeftOdometer();
//        double startRight = bot.getRightOdometer();
//
//        bot.strafeToCalib(CALIB_SPEED, desiredX, strafeDirLeft, leftReductionStrafe, rightReductionStrafe);
//
//        MotorReductionList currentList = rightReductionStrafe;
//        MotorReductionCalib currentMRCalib = currentMRRight;
//        if (strafeDirLeft){
//            currentList = leftReductionStrafe;
//            currentMRCalib = currentMRLeft;
//        }
//
//        timer.reset();
//        while(timer.milliseconds() < 2000 && opModeIsActive()){
//            telemetry.addData("Gyroscope","Stabilizing ...");
//            telemetry.update();
//        }
//
//        double endLeft = bot.getLeftOdometer();
//        double endRight = bot.getRightOdometer();
//
//        double leftDistance = Math.abs(endLeft - startLeft);
//        double rightDistance = Math.abs(endRight - startRight);
//
//
//        //get change in heading
//        double finalHead = bot.getGyroHeading();
//
//        headChange = finalHead - currentHead;
//
//
//        //if one of the directions has been calibrated, do nothing
//        if (currentMRCalib.isCalibComplete()){
//            //do nothing
//            led.OK();
//        }
//        else {
//            //set values if fully calibrated
//            if (headChange >= -2 && headChange <= 2) {
//                currentMRCalib.setCalibComplete(true);
//                led.OK();
//            } else {
//                led.needAdjustment();
//                double speedReduction = 1;
//                if (leftDistance > rightDistance){
//                    speedReduction = rightDistance/leftDistance;
//                }
//                else{
//                    speedReduction = leftDistance/rightDistance;
//                }
//                //first time
//                if (currentMRCalib.getMotorName() == MotorName.NONE){
//                    currentMRCalib.setMotorName(currentList.getFirsttMR().getMotorName());
//                    currentMRCalib.setOriginalHeadChange(headChange);
//                    currentMRCalib.setHeadChange(headChange);
//                    currentMRCalib.setMotorReduction(speedReduction);
//                }
//                else{
//                    //compare current change in heading with the previous
//                    double oldHeadChange = currentMRCalib.getHeadChange();
//                    boolean not_improved = (oldHeadChange < 0 && headChange <= oldHeadChange) || (oldHeadChange > 0 && headChange >= oldHeadChange);
//                    if (not_improved){
//                        //try another motor next time
//                        currentList.restoreList();
//                        MotorReduction next = currentList.getNextMR(currentMRCalib.getMotorName());
//                        currentMRCalib.setMotorName(next.getMotorName());
//                        currentMRCalib.setMotorReduction(speedReduction);
//                        currentMRCalib.setHeadChange(currentMRCalib.getOriginalHeadChange());
//                    }
//                    else{
//                        double diff = Math.abs(oldHeadChange - headChange);
//                        if (diff < Math.abs(oldHeadChange)){
//                            //need to reduce more
//                            double adjusted = Math.abs(headChange)/diff * currentMRCalib.getMotorReduction();
//                            currentMRCalib.setMotorReduction(adjusted);
//                        }
//                        else{
//                            //overkill. need to reduce less
//                            double adjusted = Math.abs(headChange)/diff * currentMRCalib.getMotorReduction();
//                            currentMRCalib.setMotorReduction(adjusted);
//                        }
//                    }
//                }
//                currentList.updateList(currentMRCalib.getMR());
//            }
//        }
//
//        timer.reset();
//        while(timer.milliseconds() < 3000 && opModeIsActive()){
//            telemetry.addData("Cycle","Complete");
//            telemetry.addData("strafeDirLeft", strafeDirLeft);
//            telemetry.addData("headChange", headChange);
//            telemetry.addData("startHead", currentHead);
//            telemetry.addData("finalHead", finalHead);
//            telemetry.addData("Motor", currentMRCalib.getMotorName());
//            telemetry.addData("Reduction step", currentMRCalib.getReductionStep());
//            telemetry.addData("strafeReduction", currentMRCalib.getMotorReduction());
//            telemetry.update();
//        }
//
//
//        //change direction for the next run
//        strafeDirLeft = !strafeDirLeft;
//
//        //fix heading and prepare for the next run
//        restoreHead();
//
//        timer.reset();
//        while(timer.milliseconds() < 1000 && opModeIsActive()){
//            telemetry.addData("Strafe","Waiting for the next cycle ...");
//            telemetry.update();
//        }
//
//
//        return headChange;
//
//    }
    private void calibDiag(){
        diagBot(templateDiagLeft, templateDiagRight);
//        restoreHead();
        led.none();
        //bring bot back
        bot.moveToCalib(CALIB_SPEED, CALIB_SPEED, -desiredX, templateMRBack, 0, this.led);
        saveConfigDiag(templateDiagLeft, templateDiagRight);
        showMotorReductionCalib(templateDiagLeft);
        showMotorReductionCalib(templateDiagRight);
        telemetry.update();
    }

    private void diagBot(MotorReductionBotCalib calibLeft, MotorReductionBotCalib calibRight){
        led.none();
        MotorReductionBot mrLeft = calibLeft.getMR();
        MotorReductionBot mrRight = calibRight.getMR();

        double currentHead = bot.getGyroHeading();

        double leftOdo = bot.getLeftOdometer();
        double rightOdo = bot.getRightOdometer();
        bot.diagToCalib(desiredSpeed, 0, desiredX, true, mrLeft);

        timer.reset();
        while(timer.milliseconds() < 1000 && opModeIsActive()){

        }

        double actualHead = bot.getGyroHeading();
        double headChange = Math.abs(actualHead - currentHead);

        double leftDistance = Math.abs(bot.getLeftOdometer() - leftOdo);
        double rightDistance = Math.abs(bot.getRightOdometer() - rightOdo);
        calibLeft.setLeftOdoDistanceActual(leftDistance);
        calibLeft.setRightOdoDistanceActual(rightDistance);
        calibLeft.setHeadChange(headChange);
        calibLeft.process(false);

        if (Math.abs(headChange) > MARGIN_ERROR_DEGREES){
            led.needAdjustment();
        }
        else{
            led.OK();
        }

//        restoreHead();

        timer.reset();
        while(timer.milliseconds() < 1000 && opModeIsActive()){
        }
        currentHead = bot.getGyroHeading();
        leftOdo = bot.getLeftOdometer();
        rightOdo = bot.getRightOdometer();

        bot.diagToCalib(desiredSpeed, 0, desiredX, false, mrRight);

        timer.reset();
        while(timer.milliseconds() < 1000 && opModeIsActive()){

        }

        actualHead = bot.getGyroHeading();
        headChange = Math.abs(actualHead - currentHead);

        calibRight.setHeadChange(headChange);

        if (Math.abs(headChange) > MARGIN_ERROR_DEGREES){
            led.needAdjustment();
        }
        else{
            led.OK();
        }

        leftDistance = Math.abs(bot.getLeftOdometer() - leftOdo);
        rightDistance = Math.abs(bot.getRightOdometer() - rightOdo);

        calibRight.setLeftOdoDistanceActual(leftDistance);
        calibRight.setRightOdoDistanceActual(rightDistance);
        calibRight.process(false);
    }


//    private void calibDiagMulti(){
//        try {
//            led.none();
//            desiredSpeed = CALIB_SPEED_HIGH;
//
//            MotorReductionCalib calibLeft = new MotorReductionCalib();
//            calibLeft.setCalibSpeed(desiredSpeed);
//            diagMR(true, calibLeft);
//
//            MotorReductionCalib calibRight = new MotorReductionCalib();
//            calibRight.setCalibSpeed(desiredSpeed);
//            diagMR(false, calibRight);
//
//
//            MotorReductionBot mrDiag = new MotorReductionBot();
//
//            if (calibLeft.getMotorName() == MotorName.LB){
//                mrDiag.setLB(calibLeft.getMotorReduction());
//            } else if (calibLeft.getMotorName() == MotorName.RF){
//                mrDiag.setRF(calibLeft.getMotorReduction());
//            }
//
//
//            if (calibRight.getMotorName() == MotorName.LF){
//                mrDiag.setLF(calibRight.getMotorReduction());
//            } else if (calibRight.getMotorName() == MotorName.RB){
//                mrDiag.setRB(calibRight.getMotorReduction());
//            }
//
//            //speed per degree
////            DiagCalibConfig dcLeft = diagAngle(true, calibLeft);
////            DiagCalibConfig dcRight = diagAngle(false, calibRight);
////
////            dcLeft.computeSpeedPerDegree();
////            dcRight.computeSpeedPerDegree();
//
//            telemetry.addData("Speed", desiredSpeed);
//
////            telemetry.addData("Left Angle", dcLeft.getMaxAgle());
////            telemetry.addData("Right Angle", dcRight.getMaxAgle());
////
////            telemetry.addData("Left Speed/degree", dcLeft.getSpeedPerDegree());
////            telemetry.addData("Right Speed/degree", dcRight.getSpeedPerDegree());
//
////            telemetry.addData("Left Dist", "%.3f  %.3f  %.3f", calibLeft.getBreakPointLeft(), calibLeft.getBreakPointRight(), calibLeft.getBreakPointHor());
////            telemetry.addData("Right Dist", "%.3f  %.3f  %.3f", calibRight.getBreakPointLeft(), calibRight.getBreakPointRight(), calibRight.getBreakPointHor());
////
//
//            saveConfigDiag(mrDiag);
//            showMotorReduction( mrDiag);
//            telemetry.update();
//
//        //max angle left/right
//        //speed per degree left/right
//        //speed adjustment
//            //when to break
//        }
//        catch (Exception ex){
//            telemetry.addData("Error", ex.getMessage());
//            telemetry.update();
//        }
//    }
//
//    private void diagMR(boolean left, MotorReductionCalib calib)
//    {
//        while (!calib.isCalibComplete()) {
//            double distanceInches = desiredX;
//
//
//            double leftOdoStart = bot.getLeftOdometer();
//            double rightOdoStart = bot.getRightOdometer();
//            double horOdoStart = bot.getHorizontalOdometer();
//
//            double startHead = bot.getGyroHeading();
//
////        double distance = Math.abs(distanceInches * bot.COUNTS_PER_INCH_REV);
////        double horDistance = distance * Math.sin(Math.toRadians(desiredAngle));
////        double verDistance = distance * Math.cos(Math.toRadians(desiredAngle));
////        calib.setHorOdoDistance(horDistance);
////        calib.setLeftOdoDistance(verDistance);
////        calib.setRightOdoDistance(verDistance);
//
//            bot.diagToCalib(calib.getCalibSpeed(), 0, distanceInches, left, calib);
//
//
//            timer.reset();
//            while (timer.milliseconds() < 2000 && opModeIsActive()) {
////                    telemetry.addData("Gyroscope", "Stabilizing ...");
////                    telemetry.update();
//            }
//
//            double leftOdoEnd = bot.getLeftOdometer();
//            double rightOdoEnd = bot.getRightOdometer();
//
//            double finalHead = bot.getGyroHeading();
//
//            double leftDistanceActual = Math.abs(leftOdoEnd - leftOdoStart);
//            double rightDistanceActual = Math.abs(rightOdoEnd - rightOdoStart);
//
//            double headChange = Math.abs(finalHead - startHead);
//            double reduction = 1;
//            if (headChange >= 2) {
//                this.led.needAdjustment();
//
//                if (leftDistanceActual > rightDistanceActual) {
//                    reduction = rightDistanceActual / leftDistanceActual;
//                    if (left) {
//                        calib.setMotorName(MotorName.LB);
//                    } else {
//                        calib.setMotorName(MotorName.LF);
//                    }
//                } else {
//                    reduction = leftDistanceActual / rightDistanceActual;
//                    if (left) {
//                        calib.setMotorName(MotorName.RF);
//                    } else {
//                        calib.setMotorName(MotorName.RB);
//                    }
//                }
//                calib.setMotorReduction(reduction);
//                calib.setHeadChange(headChange);
//                restoreHead();
//            } else {
//                calib.setCalibComplete(true);
//                this.led.OK();
//            }
//
//
//            timer.reset();
//            while (timer.milliseconds() < 2000 && opModeIsActive()) {
////                    telemetry.addData("Angle", angle);
////                    telemetry.addData("startHead", startHead);
////                    telemetry.addData("finalHead", finalHead);
////                    telemetry.addData("Diag", "About to go back ...");
////                    telemetry.update();
//            }
//
//            //go back
//            bot.diagToCalib(calib.getCalibSpeed(), 0, -distanceInches, left, null);
//
//            timer.reset();
//            while (timer.milliseconds() < 1000 && opModeIsActive()) {
////                    telemetry.addData("Gyroscope", "Stabilizing ...");
////                    telemetry.update();
//            }
//
//            restoreHead();
//            this.led.none();
//        }
//
//        calib.computeSpeedReduction();
//
//        timer.reset();
//        while (timer.milliseconds() < 2000 && opModeIsActive()) {
//
//        }
//
//    }

    private DiagCalibConfig diagAngle(boolean left, MotorReductionBot calib)
    {
        DiagCalibConfig diagConfig = new DiagCalibConfig();
        double [] speeds = new double[]{0.05, 0.1};
        for (int i = 0; i < speeds.length; i++) {
            double distanceInches = desiredX;


            double leftOdoStart = bot.getLeftOdometer();
            double rightOdoStart = bot.getRightOdometer();
            double horOdoStart = bot.getHorizontalOdometer();

            double startHead = bot.getGyroHeading();

//        double distance = Math.abs(distanceInches * bot.COUNTS_PER_INCH_REV);
//        double horDistance = distance * Math.sin(Math.toRadians(desiredAngle));
//        double verDistance = distance * Math.cos(Math.toRadians(desiredAngle));
//        calib.setHorOdoDistance(horDistance);
//        calib.setLeftOdoDistance(verDistance);
//        calib.setRightOdoDistance(verDistance);

            bot.diagToCalib(desiredSpeed, speeds[i], distanceInches, left, calib);


            timer.reset();
            while (timer.milliseconds() < 2000 && opModeIsActive()) {
//                    telemetry.addData("Gyroscope", "Stabilizing ...");
//                    telemetry.update();
            }

            double leftOdoEnd = bot.getLeftOdometer();
            double rightOdoEnd = bot.getRightOdometer();
            double horOdoEnd = bot.getHorizontalOdometer();

            double finalHead = bot.getGyroHeading();

            double leftDistanceActual = Math.abs(leftOdoEnd - leftOdoStart);
            double rightDistanceActual = Math.abs(rightOdoEnd - rightOdoStart);
            double horDistanceActual = Math.abs(horOdoEnd - horOdoStart);

            double headChange = Math.abs(finalHead - startHead);


//        calib.setLeftOdoDistanceActual(leftDistanceActual);
//        calib.setRightOdoDistanceActual(rightDistanceActual);
//        calib.setHorOdoDistanceActual(horDistanceActual);

            double averageVerticalDistance = (leftDistanceActual + rightDistanceActual) / 2;

            double actualAngle = Math.toDegrees(Math.atan(horDistanceActual / averageVerticalDistance));
            diagConfig.setSpeedDegreeData(speeds[i], actualAngle);

            timer.reset();
            while (timer.milliseconds() < 2000 && opModeIsActive()) {
//                    telemetry.addData("Angle", angle);
//                    telemetry.addData("startHead", startHead);
//                    telemetry.addData("finalHead", finalHead);
//                    telemetry.addData("Diag", "About to go back ...");
//                    telemetry.update();
            }

            //go back
            bot.diagToCalib(desiredSpeed, speeds[i], -distanceInches, left, null);

            timer.reset();
            while (timer.milliseconds() < 1000 && opModeIsActive()) {
//                    telemetry.addData("Gyroscope", "Stabilizing ...");
//                    telemetry.update();
            }

//            restoreHead();
            this.led.none();
        }

        timer.reset();
        while (timer.milliseconds() < 2000 && opModeIsActive()) {

        }

        return diagConfig;
    }

//    private void restoreHead(){
//        this.bot.spinH(0, 0.1);
//    }


    private void saveMoveConfigFromAmps(){
        templateMRForward.CopyMR(templateMRForward.getStats().getSuggestedMR());
        templateMRBack.CopyMR(templateMRBack.getStats().getSuggestedMR());
        saveConfigMoveForward(templateMRForward);
        saveConfigMoveBack(templateMRBack);

        telemetry.addData("Config", "Saved the following configs:");
        showMotorReductionCalib(templateMRForward);
        showMotorReductionCalib(templateMRBack);
        telemetry.update();
    }

    private void saveCurrentConfig(){
        saveConfigMoveForward(templateMRForward);
        saveConfigMoveBack(templateMRBack);
        saveConfigStrafe(templateStrafeLeft, templateStrafeRight);
        telemetry.addData("Config", "Saved the following configs:");
        showMotorReductionCalib(templateMRForward);
        showMotorReductionCalib(templateMRBack);
        showMotorReductionCalib(templateStrafeLeft);
        showMotorReductionCalib(templateStrafeRight);
        telemetry.update();
    }

    private void saveConfigMoveForward(MotorReductionBot mrForward){
        BotCalibConfig config = bot.getCalibConfig();
        if (config == null){
            config = new BotCalibConfig();
        }

        config.setMoveMRForward(mrForward);


        ReadWriteFile.writeFile(bot.getCalibConfigFile(), config.serialize());
    }

    private void saveConfigMoveBack(MotorReductionBot mrBack){
        BotCalibConfig config = bot.getCalibConfig();
        if (config == null){
            config = new BotCalibConfig();
        }

        config.setMoveMRBack(mrBack);


        ReadWriteFile.writeFile(bot.getCalibConfigFile(), config.serialize());
    }

    private void saveConfigBreak(MotorReductionBot mrForward, MotorReductionBot mrBack){
        BotCalibConfig config = bot.getCalibConfig();
        if (config == null){
            config = new BotCalibConfig();
        }
        MotorReductionBot originalForward = config.getMoveMRForward();

        MotorReductionBot originalBack = config.getMoveMRBack();


        originalForward.updateBreakSamples(mrForward);
        originalBack.updateBreakSamples(mrBack);

        ReadWriteFile.writeFile(bot.getCalibConfigFile(), config.serialize());
    }

    private void saveConfigStrafe(MotorReductionBot mrLeft, MotorReductionBot mrRight){
        BotCalibConfig config = bot.getCalibConfig();
        if (config == null){
            config = new BotCalibConfig();
        }

//        MotorReductionBot originalLeft = config.getStrafeLeftReduction();
//
//        MotorReductionBot originalRight = config.getStrafeRightReduction();

        boolean changed = true;
        config.setStrafeLeftReduction(mrLeft);
        config.setStrafeRightReduction(mrRight);

//        if (mrLeft.compare(originalLeft)){
//            config.setStrafeLeftReduction(mrLeft);
//            changed = true;
//        }
//
//        if (mrRight.compare(originalRight)) {
//            config.setStrafeRightReduction(mrRight);
//            changed = true;
//        }

        if (changed) {
            ReadWriteFile.writeFile(bot.getCalibConfigFile(), config.serialize());
        }

    }

    private void saveConfigDiag(MotorReductionBot mrLeft, MotorReductionBot mrRight){
        BotCalibConfig config = bot.getCalibConfig();
        if (config == null){
            config = new BotCalibConfig();
        }

        MotorReductionBot originalLeft = config.getDiagMRLeft();

        MotorReductionBot originalRight = config.getDiagMRRight();

        boolean changed = false;
        if (mrLeft.compare(originalLeft)){
            config.setDiagMRLeft(mrLeft);
            changed = true;
        }

        if (mrRight.compare(originalRight)) {
            config.setDiagMRRight(mrRight);
            changed = true;
        }

        if (changed) {
            ReadWriteFile.writeFile(bot.getCalibConfigFile(), config.serialize());
        }
    }

    private void showMotorReduction(MotorReductionBot mr){
        telemetry.addData("*  ","%.2f ||--  --|| %.2f", mr.getLF(), mr.getRF());
        telemetry.addData("*  ", "             ||");
        telemetry.addData("*  ", "             ||");
        telemetry.addData("*  ","%.2f ||--  --|| %.2f", mr.getLB(), mr.getRB());
    }

    private void showMotorReductionCalib(MotorReductionBotCalib mr){
        String veer = " | ";
        if (mr.getVeer() == RobotVeer.LEFT){
            veer = "<--";
        }
        if (mr.getVeer() == RobotVeer.RIGHT){
            veer = "-->";
        }
        telemetry.addData("Calib", mr.getDirection().name());
        telemetry.addData("*  ", "%s: %.2f D/ratio; %.2f; FromTarget: %.2f", veer, mr.getHeadChange(), mr.getDistanceRatio(), mr.getDistanceFromTarget());
        telemetry.addData("*  ", "L:%.2f R: %.2f ", mr.getOverDriveLeft()/bot.COUNTS_PER_INCH_REV, mr.getOverDriveRight()/bot.COUNTS_PER_INCH_REV);
        telemetry.addData("*  ", "--------------------");
        telemetry.addData("*  ","%.2f%s||--  --||%s%.2f", mr.getLF(), mr.getSelectedIndicator(0), mr.getSelectedIndicator(1), mr.getRF());
        telemetry.addData("*  ", "             ||");
        telemetry.addData("*  ", "             ||");
        telemetry.addData("*  ","%.2f%s||--  --||%s%.2f", mr.getLB(), mr.getSelectedIndicator(3), mr.getSelectedIndicator(2), mr.getRB());
        telemetry.addData("*  ", "--------------------");
        telemetry.addData("   ", "Stats:              ");
        telemetry.addData("   ", "Speed: Top %.2f, Average: %.2f", mr.getStats().getFullSpeed(), mr.getStats().getAverageSpeed());
        telemetry.addData("   ", "Dist: T: %.2f, A: %.2f, S: %.2f, Delta: %.2f", mr.getStats().getTotalDistance(), mr.getStats().getAccelerateDistance(), mr.getStats().getSlowDownDistance(), mr.getStats().getSlowDownDelta());
        telemetry.addData("   ", "Time: T: %.2f, A: %.2f, S: %.2f", mr.getStats().getTotalTime(), mr.getStats().getAccelerateTime(), mr.getStats().getSlowDownTime());
        telemetry.addData("   ", "Velocity: L: %.2f, R: %.2f", mr.getStats().getMaxVelocityLeft(), mr.getStats().getMaxVelocityRight());
        telemetry.addData("*  ", "--------------------");
        telemetry.addData("*  ", "Motor Amps (%d samples", mr.getStats().getAmpSampleCount());
        telemetry.addData("*  ","%.2f||--  --||%.2f", mr.getStats().getMotorAmpsAverages().getLF(),  mr.getStats().getMotorAmpsAverages().getRF());
        telemetry.addData("*  ", "             ||");
        telemetry.addData("*  ", "             ||");
        telemetry.addData("*  ","%.2f||--  --||%.2f", mr.getStats().getMotorAmpsAverages().getLB(),  mr.getStats().getMotorAmpsAverages().getRB());
        telemetry.addData("*  ", "--------------------");
        telemetry.addData("*  ", "--------------------");
        telemetry.addData("*  ", "Suggested MR");
        telemetry.addData("*  ","%.2f||--  --||%.2f", mr.getStats().getSuggestedMR().getLF(),  mr.getStats().getSuggestedMR().getRF());
        telemetry.addData("*  ", "             ||");
        telemetry.addData("*  ", "             ||");
        telemetry.addData("*  ","%.2f||--  --||%.2f", mr.getStats().getSuggestedMR().getLB(),  mr.getStats().getSuggestedMR().getRB());
        telemetry.addData("*  ", "--------------------");
    }

}
