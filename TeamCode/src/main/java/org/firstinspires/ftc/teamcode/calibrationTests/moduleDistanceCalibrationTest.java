package org.firstinspires.ftc.teamcode.calibrationTests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.swerveDriveBase;
import org.firstinspires.ftc.teamcode.teamUtil.robotConfig;
import org.firstinspires.ftc.teamcode.teamUtil.robotConstants;

@TeleOp(name = "Module Distance Calibration Test", group = "Calibration Tests")
public class moduleDistanceCalibrationTest extends LinearOpMode{

    robotConfig r;
    private final ElapsedTime runTime = new ElapsedTime();

    enum testState {
        INIT_POS, SELECT_DISTANCE, DISTANCE, RETURN_VALUE, FINISHED
    }

    testState state = testState.INIT_POS;

    @Override
    public void runOpMode() throws InterruptedException {
        int mmPerTest = 1000;
        runTime.reset();
        boolean testFinished = false;
        robotConstants.configuredSystems modules = robotConstants.configuredSystems.BOTH_MODULES;
        boolean buttonPressed1 = false;
        boolean buttonPressed2 = false;
        boolean xSelected = false;
        boolean aPressed = false;
        double finalTicksLeft = 0;
        double finalTicksRight = 0;
        String finalTicksLeftString = "";
        String finalTicksRightString = "";

        waitForStart();

        while (!(state == testState.FINISHED || gamepad1.y)) {
            if(!gamepad1.a){
                aPressed = false;
            }
            if(gamepad1.b){
                state = testState.INIT_POS;
            }

            if (state == testState.INIT_POS && !gamepad1.b) {
                telemetry.addLine("line up module(s) in start position, and use the dpad left and right to select enabled modules, then press a");
                telemetry.addData("enabled module(s)", modules.name());
                telemetry.addLine("press b to reset, press y to end program");
                telemetry.update();

                switch (modules) {
                    case RIGHT_MODULE:
                        telemetry.addData("right ticks constant", robotConstants.ticksPerMMRightSwerve);
                        break;

                    case LEFT_MODULE:
                        telemetry.addData("left ticks constant", robotConstants.ticksPerMMLeftSwerve);
                        break;

                    case BOTH_MODULES:
                        telemetry.addData("left ticks constant", robotConstants.ticksPerMMLeftSwerve);
                        telemetry.addData("right ticks constant", robotConstants.ticksPerMMRightSwerve);
                        break;
                }

                if (!buttonPressed1 && gamepad1.dpad_right) {
                    switch (modules) {
                        case RIGHT_MODULE:
                            modules = robotConstants.configuredSystems.LEFT_MODULE;
                            break;
                        case LEFT_MODULE:
                            modules = robotConstants.configuredSystems.BOTH_MODULES;
                            break;
                        case BOTH_MODULES:
                            modules = robotConstants.configuredSystems.RIGHT_MODULE;
                            break;
                    }
                    buttonPressed1 = true;
                }
                else if (!buttonPressed1 && gamepad1.dpad_left) {
                    switch (modules) {
                        case RIGHT_MODULE:
                            modules = robotConstants.configuredSystems.BOTH_MODULES;
                            break;
                        case LEFT_MODULE:
                            modules = robotConstants.configuredSystems.RIGHT_MODULE;
                            break;
                        case BOTH_MODULES:
                            modules = robotConstants.configuredSystems.LEFT_MODULE;
                            break;
                    }
                    buttonPressed1 = true;
                }
                else if(!(gamepad1.dpad_right || gamepad1.dpad_left)){
                    buttonPressed1 = false;
                }

                if (gamepad1.a && !aPressed) {
                    telemetry.addData("Status", "Initialising");
                    telemetry.update();
                    r = new robotConfig(this, modules);
                    telemetry.addData("Status", "Initialised");
                    telemetry.update();
                    state = testState.SELECT_DISTANCE;
                    swerveDriveBase.resetEncoders();
                    switch (modules) {
                        case RIGHT_MODULE:
                            robotConstants.ticksPerMMRightSwerve = 1;
                            break;
                        case LEFT_MODULE:
                            robotConstants.ticksPerMMLeftSwerve = 1;
                            break;
                        case BOTH_MODULES:
                            robotConstants.ticksPerMMLeftSwerve = 1;
                            robotConstants.ticksPerMMRightSwerve = 1;
                            break;
                    }
                    aPressed = true;
                }
            }
            if (state == testState.SELECT_DISTANCE && !gamepad1.b) {
                r.encoderRead.encoderBulkRead();
                telemetry.addLine("use dpad up and down to change the distance to be used for calibration in the test, use dpad left and right to select if testing for x or y position, press a when finished");
                telemetry.addData("distance to be tested over (mm)", mmPerTest);
                if(xSelected){
                    telemetry.addLine("will test for the x axis");
                    switch (swerveDriveBase.enabledModules) {
                        case LEFT:
                            telemetry.addData("distance travelled left x", swerveDriveBase.leftPose2D.coordinate2D.x);
                            break;

                        case RIGHT:
                            telemetry.addData("distance travelled right x", swerveDriveBase.rightPose2D.coordinate2D.x);
                            break;

                        case BOTH:
                            telemetry.addData("distance travelled left x", swerveDriveBase.leftPose2D.coordinate2D.x);
                            telemetry.addData("distance travelled right x", swerveDriveBase.rightPose2D.coordinate2D.x);
                            break;
                    }
                }
                else{
                    telemetry.addLine("will test for the y axis");
                    switch (swerveDriveBase.enabledModules) {
                        case LEFT:
                            telemetry.addData("distance travelled left y", swerveDriveBase.leftPose2D.coordinate2D.y);
                            break;

                        case RIGHT:
                            telemetry.addData("distance travelled right y", swerveDriveBase.rightPose2D.coordinate2D.y);
                            break;

                        case BOTH:
                            telemetry.addData("distance travelled left y", swerveDriveBase.leftPose2D.coordinate2D.y);
                            telemetry.addData("distance travelled right y", swerveDriveBase.rightPose2D.coordinate2D.y);
                            break;
                    }
                }

                telemetry.addLine("press b to reset, press y to end program");
                telemetry.update();

                if(!buttonPressed1 && (gamepad1.dpad_left || gamepad1.dpad_right)){
                    xSelected = !xSelected;
                    buttonPressed1 = true;
                }
                else if (!(gamepad1.dpad_left || gamepad1.dpad_right)){
                    buttonPressed1 = false;
                }

                if (!buttonPressed2 && mmPerTest >= 100 && gamepad1.dpad_down) {
                    mmPerTest = -100;
                    buttonPressed2 = true;
                }
                else if (!buttonPressed2 && gamepad1.dpad_up) {
                    mmPerTest += 100;
                    buttonPressed2 = true;
                }
                else if (!(gamepad1.dpad_up || gamepad1.dpad_down)){
                    buttonPressed2 = false;
                }

                if (gamepad1.a && !aPressed) {
                    state = testState.DISTANCE;
                    aPressed = true;
                }
            }
            if (state == testState.DISTANCE && !gamepad1.b) {
                r.encoderRead.encoderBulkRead();
                telemetry.addLine("press b to reset, press y to end program");
                if(xSelected){
                    telemetry.addLine("move the robot along the x axis " + mmPerTest + "mm, press a when finished");
                    switch (swerveDriveBase.enabledModules) {
                        case LEFT:
                            telemetry.addData("distance travelled left x", swerveDriveBase.leftPose2D.coordinate2D.x);
                            break;

                        case RIGHT:
                            telemetry.addData("distance travelled right x", swerveDriveBase.rightPose2D.coordinate2D.x);
                            break;

                        case BOTH:
                            telemetry.addData("distance travelled left x", swerveDriveBase.leftPose2D.coordinate2D.x);
                            telemetry.addData("distance travelled right x", swerveDriveBase.rightPose2D.coordinate2D.x);
                            break;
                    }
                }
                else{
                    telemetry.addLine("move the robot along the y axis " + mmPerTest + "mm, press a when finished");
                    switch (swerveDriveBase.enabledModules) {
                        case LEFT:
                            telemetry.addData("distance travelled left y", swerveDriveBase.leftPose2D.coordinate2D.y);
                            break;

                        case RIGHT:
                            telemetry.addData("distance travelled right y", swerveDriveBase.rightPose2D.coordinate2D.y);
                            break;

                        case BOTH:
                            telemetry.addData("distance travelled left y", swerveDriveBase.leftPose2D.coordinate2D.y);
                            telemetry.addData("distance travelled right y", swerveDriveBase.rightPose2D.coordinate2D.y);
                            break;
                    }
                }
                telemetry.update();

                if (gamepad1.a && !aPressed) {
                    state = testState.RETURN_VALUE;
                    testFinished = false;
                    aPressed = true;
                }
            }

            if (state == testState.RETURN_VALUE && !gamepad1.b) {
                r.encoderRead.encoderBulkRead();
                if (!testFinished) {
                    if(xSelected) {
                        switch (swerveDriveBase.enabledModules) {
                            case LEFT:
                                finalTicksLeft = returnTestResults(swerveDriveBase.leftPose2D.coordinate2D.x, mmPerTest);
                                finalTicksLeftString = testResultsString(swerveDriveBase.leftPose2D.coordinate2D.x, mmPerTest);
                                robotConstants.ticksPerMMLeftSwerve = finalTicksLeft;
                                break;

                            case RIGHT:
                                finalTicksRight = returnTestResults(swerveDriveBase.rightPose2D.coordinate2D.x, mmPerTest);
                                finalTicksRightString = testResultsString(swerveDriveBase.rightPose2D.coordinate2D.x, mmPerTest);
                                robotConstants.ticksPerMMRightSwerve = finalTicksRight;
                                break;

                            case BOTH:
                                finalTicksLeft = returnTestResults(swerveDriveBase.leftPose2D.coordinate2D.x, mmPerTest);
                                finalTicksRight = returnTestResults(swerveDriveBase.rightPose2D.coordinate2D.x, mmPerTest);
                                finalTicksLeftString = testResultsString(swerveDriveBase.leftPose2D.coordinate2D.x, mmPerTest);
                                finalTicksRightString = testResultsString(swerveDriveBase.rightPose2D.coordinate2D.x, mmPerTest);
                                robotConstants.ticksPerMMLeftSwerve = finalTicksLeft;
                                robotConstants.ticksPerMMRightSwerve = finalTicksRight;
                                break;
                        }
                    }
                    else{
                        switch (swerveDriveBase.enabledModules) {
                            case LEFT:
                                finalTicksLeft = returnTestResults(swerveDriveBase.leftPose2D.coordinate2D.y, mmPerTest);
                                finalTicksLeftString = testResultsString(swerveDriveBase.leftPose2D.coordinate2D.y, mmPerTest);
                                robotConstants.ticksPerMMLeftSwerve = finalTicksLeft;
                                break;

                            case RIGHT:
                                finalTicksRight = returnTestResults(swerveDriveBase.rightPose2D.coordinate2D.y, mmPerTest);
                                finalTicksRightString = testResultsString(swerveDriveBase.rightPose2D.coordinate2D.y, mmPerTest);
                                robotConstants.ticksPerMMRightSwerve = finalTicksRight;
                                break;

                            case BOTH:
                                finalTicksLeft = returnTestResults(swerveDriveBase.leftPose2D.coordinate2D.y, mmPerTest);
                                finalTicksRight = returnTestResults(swerveDriveBase.rightPose2D.coordinate2D.y, mmPerTest);
                                finalTicksLeftString = testResultsString(swerveDriveBase.leftPose2D.coordinate2D.y, mmPerTest);
                                finalTicksRightString = testResultsString(swerveDriveBase.rightPose2D.coordinate2D.y, mmPerTest);
                                robotConstants.ticksPerMMLeftSwerve = finalTicksLeft;
                                robotConstants.ticksPerMMRightSwerve = finalTicksRight;
                                break;
                        }
                    }
                    testFinished = true;
                }
                switch (swerveDriveBase.enabledModules) {
                    case LEFT:
                        telemetry.addData("final value left module: " + finalTicksLeftString, finalTicksLeft);
                        break;

                    case RIGHT:
                        telemetry.addData("final value right module: " + finalTicksRightString, finalTicksRight);
                        break;

                    case BOTH:
                        telemetry.addData("final value left module: " + finalTicksLeftString, finalTicksLeft);
                        telemetry.addData("final value right module: " + finalTicksRightString, finalTicksRight);
                        break;
                }

                telemetry.addLine("press a to finish, or b to restart");
                telemetry.update();

                if (gamepad1.a && !aPressed) {
                    state = testState.FINISHED;
                    aPressed = true;
                }
            }
        }
    }

    String testResultsString(double finalTicks, int distanceTraveled){
        return Math.abs(finalTicks) + "/" + (distanceTraveled);
    }

    double returnTestResults(double finalTicks, int distanceTraveled){
        return (Math.abs(finalTicks)/(distanceTraveled));
    }
}
