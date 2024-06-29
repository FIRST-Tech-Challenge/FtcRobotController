package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class TestTeleOpp extends LinearOpMode {

    Robot robot;

    DroneLauncherState droneLauncherState;
    IntakeState intakeState;
    OuttakeState outtakeState;

    DroneLauncher droneLauncher;
    Intake intake;
    Outtake outtake;

    TrueState trueState;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap, robot.opMode, telemetry, false, false, false);

        while (opModeIsActive()) {
            //setting current and target
            robot.outtake.setState(robot.lsFront.getCurrentPosition(), 500, outtakeState);
            robot.intake.setState(robot.intakeMotor.getCurrentPosition(), 500,  intakeState);
            robot.droneLauncher.setState(robot.planeLauncher.getCurrentPosition(), 1000,droneLauncherState);

            //sequence 1
            robot.intake.wheelMotor.setPower(robot.intake.intakeTestSequencePower(trueState, intakeState));
            robot.outtake.lsFront.setPower(robot.outtake.lsParallelPowerPSequence(intakeState, outtakeState, 1));
            robot.outtake.lsBack.setPower(-1 * (robot.outtake.lsParallelPowerPSequence(intakeState, outtakeState, 1)));

            //sequence 2
            robot.droneLauncher.testDroneMotorSequencePower(trueState, droneLauncherState);
        }

//        robot.lsFront.setPower(0.001 * (outtakeState.getError()));
//        robot.intake.setPower(0.001*(intakeState.getError()));
//        robot.planeLauncher.setPower(0.001*(droneLauncherState.getError()));

    }
}
