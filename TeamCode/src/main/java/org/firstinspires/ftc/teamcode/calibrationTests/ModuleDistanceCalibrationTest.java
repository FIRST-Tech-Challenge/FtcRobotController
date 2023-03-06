//package org.firstinspires.ftc.teamcode.calibrationTests;
//
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.teamcode.subsystems.swerve.SwerveDriveBase;
//import org.firstinspires.ftc.teamcode.teamUtil.RobotConfig;
//import org.firstinspires.ftc.teamcode.teamUtil.RobotConstants;
//
//@TeleOp(name = "Module Distance Calibration Test", group = "Calibration Tests")
//public class ModuleDistanceCalibrationTest extends LinearOpMode{
//
//    RobotConfig r;
//    private final ElapsedTime runTime = new ElapsedTime();
//
//    enum testState {
//        INIT_POS, SELECT_DISTANCE, DISTANCE, RETURN_VALUE, FINISHED
//    }
//
//    testState state = testState.INIT_POS;
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        int mmPerTest = 1000;
//        runTime.reset();
//        boolean testFinished = false;
//        RobotConstants.configuredSystems modules = RobotConstants.configuredSystems.BOTH_MODULES;
//        boolean buttonPressed1 = false;
//        boolean buttonPressed2 = false;
//        boolean xSelected = false;
//        boolean aPressed = false;
//        double finalTicksLeft = 0;
//        double finalTicksRight = 0;
//        String finalTicksLeftString = "";
//        String finalTicksRightString = "";
//        encoderRead = r.getSubsystem(RobotConstants.configuredSystems.ENCODER_READ);
//
//        waitForStart();
//
//        while (!(state == testState.FINISHED || gamepad1.y)) {
//            if(!gamepad1.a){
//                aPressed = false;
//            }
//            if(gamepad1.b){
//                state = testState.INIT_POS;
//            }
//
//            if (state == testState.INIT_POS && !gamepad1.b) {
//                telemetry.addLine("line up module(s) in start position, and use the dpad left and right to select enabled modules, then press a");
//                telemetry.addData("enabled module(s)", modules.name());
//                telemetry.addLine("press b to reset, press y to end program");
//                telemetry.update();
//
//                switch (modules) {
//                    case RIGHT_MODULE:
//                        telemetry.addData("right ticks constant", RobotConstants.ticksPerMMRightSwerve);
//                        break;
//
//                    case LEFT_MODULE:
//                        telemetry.addData("left ticks constant", RobotConstants.ticksPerMMLeftSwerve);
//                        break;
//
//                    case BOTH_MODULES:
//                        telemetry.addData("left ticks constant", RobotConstants.ticksPerMMLeftSwerve);
//                        telemetry.addData("right ticks constant", RobotConstants.ticksPerMMRightSwerve);
//                        break;
//                }
//
//                if (!buttonPressed1 && gamepad1.dpad_right) {
//                    switch (modules) {
//                        case RIGHT_MODULE:
//                            modules = RobotConstants.configuredSystems.LEFT_MODULE;
//                            break;
//                        case LEFT_MODULE:
//                            modules = RobotConstants.configuredSystems.BOTH_MODULES;
//                            break;
//                        case BOTH_MODULES:
//                            modules = RobotConstants.configuredSystems.RIGHT_MODULE;
//                            break;
//                    }
//                    buttonPressed1 = true;
//                }
//                else if (!buttonPressed1 && gamepad1.dpad_left) {
//                    switch (modules) {
//                        case RIGHT_MODULE:
//                            modules = RobotConstants.configuredSystems.BOTH_MODULES;
//                            break;
//                        case LEFT_MODULE:
//                            modules = RobotConstants.configuredSystems.RIGHT_MODULE;
//                            break;
//                        case BOTH_MODULES:
//                            modules = RobotConstants.configuredSystems.LEFT_MODULE;
//                            break;
//                    }
//                    buttonPressed1 = true;
//                }
//                else if(!(gamepad1.dpad_right || gamepad1.dpad_left)){
//                    buttonPressed1 = false;
//                }
//
//                if (gamepad1.a && !aPressed) {
//                    telemetry.addData("Status", "Initialising");
//                    telemetry.update();
//                    r = RobotConfig.getInstance(this);
//                    r.initSystems(modules);
//                    telemetry.addData("Status", "Initialised");
//                    telemetry.update();
//                    state = testState.SELECT_DISTANCE;
//                    SwerveDriveBase.resetEncoders();
//                    switch (modules) {
//                        case RIGHT_MODULE:
//                            RobotConstants.ticksPerMMRightSwerve = 1;
//                            break;
//                        case LEFT_MODULE:
//                            RobotConstants.ticksPerMMLeftSwerve = 1;
//                            break;
//                        case BOTH_MODULES:
//                            RobotConstants.ticksPerMMLeftSwerve = 1;
//                            RobotConstants.ticksPerMMRightSwerve = 1;
//                            break;
//                    }
//                    aPressed = true;
//                }
//            }
//            if (state == testState.SELECT_DISTANCE && !gamepad1.b) {
//                encoderRead.encoderBulkRead();
//                telemetry.addLine("use dpad up and down to change the distance to be used for calibration in the test, use dpad left and right to select if testing for x or y position, press a when finished");
//                telemetry.addData("distance to be tested over (mm)", mmPerTest);
//                if(xSelected){
//                    telemetry.addLine("will test for the x axis");
//                    switch (SwerveDriveBase.enabledModules) {
//                        case LEFT:
//                            telemetry.addData("distance travelled left x", SwerveDriveBase.leftPose2D.coordinate2D.x);
//                            break;
//
//                        case RIGHT:
//                            telemetry.addData("distance travelled right x", SwerveDriveBase.rightPose2D.coordinate2D.x);
//                            break;
//
//                        case BOTH:
//                            telemetry.addData("distance travelled left x", SwerveDriveBase.leftPose2D.coordinate2D.x);
//                            telemetry.addData("distance travelled right x", SwerveDriveBase.rightPose2D.coordinate2D.x);
//                            break;
//                    }
//                }
//                else{
//                    telemetry.addLine("will test for the y axis");
//                    switch (SwerveDriveBase.enabledModules) {
//                        case LEFT:
//                            telemetry.addData("distance travelled left y", SwerveDriveBase.leftPose2D.coordinate2D.y);
//                            break;
//
//                        case RIGHT:
//                            telemetry.addData("distance travelled right y", SwerveDriveBase.rightPose2D.coordinate2D.y);
//                            break;
//
//                        case BOTH:
//                            telemetry.addData("distance travelled left y", SwerveDriveBase.leftPose2D.coordinate2D.y);
//                            telemetry.addData("distance travelled right y", SwerveDriveBase.rightPose2D.coordinate2D.y);
//                            break;
//                    }
//                }
//
//                telemetry.addLine("press b to reset, press y to end program");
//                telemetry.update();
//
//                if(!buttonPressed1 && (gamepad1.dpad_left || gamepad1.dpad_right)){
//                    xSelected = !xSelected;
//                    buttonPressed1 = true;
//                }
//                else if (!(gamepad1.dpad_left || gamepad1.dpad_right)){
//                    buttonPressed1 = false;
//                }
//
//                if (!buttonPressed2 && mmPerTest >= 100 && gamepad1.dpad_down) {
//                    mmPerTest = -100;
//                    buttonPressed2 = true;
//                }
//                else if (!buttonPressed2 && gamepad1.dpad_up) {
//                    mmPerTest += 100;
//                    buttonPressed2 = true;
//                }
//                else if (!(gamepad1.dpad_up || gamepad1.dpad_down)){
//                    buttonPressed2 = false;
//                }
//
//                if (gamepad1.a && !aPressed) {
//                    state = testState.DISTANCE;
//                    aPressed = true;
//                }
//            }
//            if (state == testState.DISTANCE && !gamepad1.b) {
//                encoderRead.encoderBulkRead();
//                telemetry.addLine("press b to reset, press y to end program");
//                if(xSelected){
//                    telemetry.addLine("move the robot along the x axis " + mmPerTest + "mm, press a when finished");
//                    switch (SwerveDriveBase.enabledModules) {
//                        case LEFT:
//                            telemetry.addData("distance travelled left x", SwerveDriveBase.leftPose2D.coordinate2D.x);
//                            break;
//
//                        case RIGHT:
//                            telemetry.addData("distance travelled right x", SwerveDriveBase.rightPose2D.coordinate2D.x);
//                            break;
//
//                        case BOTH:
//                            telemetry.addData("distance travelled left x", SwerveDriveBase.leftPose2D.coordinate2D.x);
//                            telemetry.addData("distance travelled right x", SwerveDriveBase.rightPose2D.coordinate2D.x);
//                            break;
//                    }
//                }
//                else{
//                    telemetry.addLine("move the robot along the y axis " + mmPerTest + "mm, press a when finished");
//                    switch (SwerveDriveBase.enabledModules) {
//                        case LEFT:
//                            telemetry.addData("distance travelled left y", SwerveDriveBase.leftPose2D.coordinate2D.y);
//                            break;
//
//                        case RIGHT:
//                            telemetry.addData("distance travelled right y", SwerveDriveBase.rightPose2D.coordinate2D.y);
//                            break;
//
//                        case BOTH:
//                            telemetry.addData("distance travelled left y", SwerveDriveBase.leftPose2D.coordinate2D.y);
//                            telemetry.addData("distance travelled right y", SwerveDriveBase.rightPose2D.coordinate2D.y);
//                            break;
//                    }
//                }
//                telemetry.update();
//
//                if (gamepad1.a && !aPressed) {
//                    state = testState.RETURN_VALUE;
//                    testFinished = false;
//                    aPressed = true;
//                }
//            }
//
//            if (state == testState.RETURN_VALUE && !gamepad1.b) {
//                encoderRead.encoderBulkRead();
//                if (!testFinished) {
//                    if(xSelected) {
//                        switch (SwerveDriveBase.enabledModules) {
//                            case LEFT:
//                                finalTicksLeft = returnTestResults(SwerveDriveBase.leftPose2D.coordinate2D.x, mmPerTest);
//                                finalTicksLeftString = testResultsString(SwerveDriveBase.leftPose2D.coordinate2D.x, mmPerTest);
//                                RobotConstants.ticksPerMMLeftSwerve = finalTicksLeft;
//                                break;
//
//                            case RIGHT:
//                                finalTicksRight = returnTestResults(SwerveDriveBase.rightPose2D.coordinate2D.x, mmPerTest);
//                                finalTicksRightString = testResultsString(SwerveDriveBase.rightPose2D.coordinate2D.x, mmPerTest);
//                                RobotConstants.ticksPerMMRightSwerve = finalTicksRight;
//                                break;
//
//                            case BOTH:
//                                finalTicksLeft = returnTestResults(SwerveDriveBase.leftPose2D.coordinate2D.x, mmPerTest);
//                                finalTicksRight = returnTestResults(SwerveDriveBase.rightPose2D.coordinate2D.x, mmPerTest);
//                                finalTicksLeftString = testResultsString(SwerveDriveBase.leftPose2D.coordinate2D.x, mmPerTest);
//                                finalTicksRightString = testResultsString(SwerveDriveBase.rightPose2D.coordinate2D.x, mmPerTest);
//                                RobotConstants.ticksPerMMLeftSwerve = finalTicksLeft;
//                                RobotConstants.ticksPerMMRightSwerve = finalTicksRight;
//                                break;
//                        }
//                    }
//                    else{
//                        switch (SwerveDriveBase.enabledModules) {
//                            case LEFT:
//                                finalTicksLeft = returnTestResults(SwerveDriveBase.leftPose2D.coordinate2D.y, mmPerTest);
//                                finalTicksLeftString = testResultsString(SwerveDriveBase.leftPose2D.coordinate2D.y, mmPerTest);
//                                RobotConstants.ticksPerMMLeftSwerve = finalTicksLeft;
//                                break;
//
//                            case RIGHT:
//                                finalTicksRight = returnTestResults(SwerveDriveBase.rightPose2D.coordinate2D.y, mmPerTest);
//                                finalTicksRightString = testResultsString(SwerveDriveBase.rightPose2D.coordinate2D.y, mmPerTest);
//                                RobotConstants.ticksPerMMRightSwerve = finalTicksRight;
//                                break;
//
//                            case BOTH:
//                                finalTicksLeft = returnTestResults(SwerveDriveBase.leftPose2D.coordinate2D.y, mmPerTest);
//                                finalTicksRight = returnTestResults(SwerveDriveBase.rightPose2D.coordinate2D.y, mmPerTest);
//                                finalTicksLeftString = testResultsString(SwerveDriveBase.leftPose2D.coordinate2D.y, mmPerTest);
//                                finalTicksRightString = testResultsString(SwerveDriveBase.rightPose2D.coordinate2D.y, mmPerTest);
//                                RobotConstants.ticksPerMMLeftSwerve = finalTicksLeft;
//                                RobotConstants.ticksPerMMRightSwerve = finalTicksRight;
//                                break;
//                        }
//                    }
//                    testFinished = true;
//                }
//                switch (SwerveDriveBase.enabledModules) {
//                    case LEFT:
//                        telemetry.addData("final value left module: " + finalTicksLeftString, finalTicksLeft);
//                        break;
//
//                    case RIGHT:
//                        telemetry.addData("final value right module: " + finalTicksRightString, finalTicksRight);
//                        break;
//
//                    case BOTH:
//                        telemetry.addData("final value left module: " + finalTicksLeftString, finalTicksLeft);
//                        telemetry.addData("final value right module: " + finalTicksRightString, finalTicksRight);
//                        break;
//                }
//
//                telemetry.addLine("press a to finish, or b to restart");
//                telemetry.update();
//
//                if (gamepad1.a && !aPressed) {
//                    state = testState.FINISHED;
//                    aPressed = true;
//                }
//            }
//        }
//    }
//
//    String testResultsString(double finalTicks, int distanceTraveled){
//        return Math.abs(finalTicks) + "/" + (distanceTraveled);
//    }
//
//    double returnTestResults(double finalTicks, int distanceTraveled){
//        return (Math.abs(finalTicks)/(distanceTraveled));
//    }
//}
