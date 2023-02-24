package org.firstinspires.ftc.teamcode.calibrationTests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.SwerveDriveBase;
import org.firstinspires.ftc.teamcode.teamUtil.Angle;
import org.firstinspires.ftc.teamcode.teamUtil.EncoderRead;
import org.firstinspires.ftc.teamcode.teamUtil.RobotConfig;
import org.firstinspires.ftc.teamcode.teamUtil.RobotConstants;
@Disabled
@TeleOp(name = "Module Angle Calibration Test", group = "Calibration Tests")
public class ModuleAngleCalibrationTest extends LinearOpMode{

    RobotConfig r;
    private final ElapsedTime runTime = new ElapsedTime();

    enum testState {
        INIT_POS, SELECT_ROTATIONS, ROTATION, RETURN_VALUE, FINISHED
    }

    EncoderRead encoderRead;

    testState state = testState.INIT_POS;

    @Override
    public void runOpMode() throws InterruptedException {
        int rotationsPerTest = 1;
        runTime.reset();
        boolean testFinished = false;
        RobotConstants.configuredSystems modules = RobotConstants.configuredSystems.BOTH_MODULES;
        boolean buttonPressed1 = false;
        boolean buttonPressed2 = false;
        boolean aPressed = false;
        double finalTicksLeft = 0;
        double finalTicksRight = 0;
        String finalTicksLeftString = "";
        String finalTicksRightString = "";
        encoderRead = r.getSubsystem(RobotConstants.configuredSystems.ENCODER_READ);


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
                        telemetry.addData("right ticks constant", RobotConstants.ticksPerDegreeRightSwerve);
                        break;

                    case LEFT_MODULE:
                        telemetry.addData("left ticks constant", RobotConstants.ticksPerDegreeLeftSwerve);
                        break;

                    case BOTH_MODULES:
                        telemetry.addData("left ticks constant", RobotConstants.ticksPerDegreeLeftSwerve);
                        telemetry.addData("right ticks constant", RobotConstants.ticksPerDegreeRightSwerve);
                        break;
                }

                if (!buttonPressed1 && gamepad1.dpad_right) {
                    switch (modules) {
                        case RIGHT_MODULE:
                            modules = RobotConstants.configuredSystems.LEFT_MODULE;
                            break;
                        case LEFT_MODULE:
                            modules = RobotConstants.configuredSystems.BOTH_MODULES;
                            break;
                        case BOTH_MODULES:
                            modules = RobotConstants.configuredSystems.RIGHT_MODULE;
                            break;
                    }
                    buttonPressed1 = true;
                }
                else if (!buttonPressed1 && gamepad1.dpad_left) {
                    switch (modules) {
                        case RIGHT_MODULE:
                            modules = RobotConstants.configuredSystems.BOTH_MODULES;
                            break;
                        case LEFT_MODULE:
                            modules = RobotConstants.configuredSystems.RIGHT_MODULE;
                            break;
                        case BOTH_MODULES:
                            modules = RobotConstants.configuredSystems.LEFT_MODULE;
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
                    r = RobotConfig.getInstance(this);
                    r.initSystems(modules);
                    telemetry.addData("Status", "Initialised");
                    telemetry.update();
                    state = testState.SELECT_ROTATIONS;
                    SwerveDriveBase.resetEncoders();
                    switch (modules) {
                        case RIGHT_MODULE:
                            RobotConstants.ticksPerDegreeRightSwerve = 1;
                            break;
                        case LEFT_MODULE:
                            RobotConstants.ticksPerDegreeLeftSwerve = 1;
                            break;
                        case BOTH_MODULES:
                            RobotConstants.ticksPerDegreeLeftSwerve = 1;
                            RobotConstants.ticksPerDegreeRightSwerve = 1;
                            break;
                    }
                    aPressed = true;
                }
            }
            if (state == testState.SELECT_ROTATIONS && !gamepad1.b) {
                encoderRead.encoderBulkRead();
                telemetry.addLine("use dpad up and down to change the number of rotations in the test, press a when finished");
                telemetry.addData("rotations to be done", rotationsPerTest);
                switch (SwerveDriveBase.enabledModules) {
                    case LEFT:
                        telemetry.addData("continuous rotation angle left", SwerveDriveBase.left.continuousAngle.value);
                        break;
                    case RIGHT:
                        telemetry.addData("continuous rotation angle right", SwerveDriveBase.right.continuousAngle.value);
                        break;
                    case BOTH:
                        telemetry.addData("continuous rotation angle left", SwerveDriveBase.left.continuousAngle.value);
                        telemetry.addData("continuous rotation angle right", SwerveDriveBase.right.continuousAngle.value);
                        break;
                }
                telemetry.addLine("press b to reset, press y to end program");
                telemetry.update();


                if (!buttonPressed2 && rotationsPerTest >= 1 && gamepad1.dpad_down) {
                    rotationsPerTest = -1;
                    buttonPressed2 = true;
                }
                else if (!buttonPressed2 && gamepad1.dpad_up) {
                    rotationsPerTest += 1;
                    buttonPressed2 = true;
                }
                else if (!(gamepad1.dpad_up || gamepad1.dpad_down)){
                    buttonPressed2 = false;
                }

                if (gamepad1.a && !aPressed) {
                    state = testState.ROTATION;
                    aPressed = true;
                }
            }
            if (state == testState.ROTATION && !gamepad1.b) {
                encoderRead.encoderBulkRead();
                telemetry.addLine("rotate the module by hand " + rotationsPerTest + " times, press a when finished");
                telemetry.addLine("press b to reset, press y to end program");
                switch (SwerveDriveBase.enabledModules) {
                    case LEFT:
                        telemetry.addData("continuous rotation angle left", SwerveDriveBase.left.continuousAngle.value);
                        break;
                    case RIGHT:
                        telemetry.addData("continuous rotation angle right", SwerveDriveBase.right.continuousAngle.value);
                        break;
                    case BOTH:
                        telemetry.addData("continuous rotation angle left", SwerveDriveBase.left.continuousAngle.value);
                        telemetry.addData("continuous rotation angle right", SwerveDriveBase.right.continuousAngle.value);
                        break;
                }
                telemetry.update();

                if (gamepad1.a && !aPressed) {
                    state = testState.RETURN_VALUE;
                    testFinished = false;
                    aPressed = true;
                }
            }

            if (state == testState.RETURN_VALUE && !gamepad1.b) {
                encoderRead.encoderBulkRead();
                if (!testFinished) {
                    switch (SwerveDriveBase.enabledModules) {
                        case LEFT:
                            finalTicksLeft = returnTestResults(SwerveDriveBase.left.continuousAngle, rotationsPerTest);
                            finalTicksLeftString = testResultsString(SwerveDriveBase.left.continuousAngle, rotationsPerTest);
                            RobotConstants.ticksPerDegreeLeftSwerve = finalTicksLeft;
                            break;

                        case RIGHT:
                            finalTicksRight = returnTestResults(SwerveDriveBase.right.continuousAngle, rotationsPerTest);
                            finalTicksRightString = testResultsString(SwerveDriveBase.right.continuousAngle, rotationsPerTest);
                            RobotConstants.ticksPerDegreeRightSwerve = finalTicksRight;
                            break;

                        case BOTH:
                            finalTicksLeft = returnTestResults(SwerveDriveBase.left.continuousAngle, rotationsPerTest);
                            finalTicksRight = returnTestResults(SwerveDriveBase.right.continuousAngle, rotationsPerTest);
                            finalTicksLeftString = testResultsString(SwerveDriveBase.left.continuousAngle, rotationsPerTest);
                            finalTicksRightString = testResultsString(SwerveDriveBase.right.continuousAngle, rotationsPerTest);
                            RobotConstants.ticksPerDegreeLeftSwerve = finalTicksLeft;
                            RobotConstants.ticksPerDegreeRightSwerve = finalTicksRight;
                            break;
                    }
                    testFinished = true;
                }
                switch (SwerveDriveBase.enabledModules) {
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

    String testResultsString(Angle finalRotation, int rotationsDone){
        return Math.abs(finalRotation.value) + "/" + (rotationsDone * 360);
    }

    double returnTestResults(Angle finalRotation, int rotationsDone){
        return (Math.abs(finalRotation.value)/(rotationsDone*360));
    }
}
