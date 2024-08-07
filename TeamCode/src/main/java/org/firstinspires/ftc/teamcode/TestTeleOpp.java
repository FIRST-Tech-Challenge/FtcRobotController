package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
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
        robot = new Robot(hardwareMap, this, telemetry, false, false, false);
        outtakeState = new OuttakeState();
        intakeState = new IntakeState();
        droneLauncherState = new DroneLauncherState();
        trueState = new TrueState();

        outtakeState.setOpMode(robot.opMode);
        intakeState.setOpMode(robot.opMode);
        droneLauncherState.setOpMode(robot.opMode);

        waitForStart();

        robot.initForTeleOp();


        robot.intake.wheelMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.intake.wheelMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        robot.lsFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.lsFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        while (opModeIsActive()) {

            //setting current and target
            robot.outtake.setState(robot.lsFront.getCurrentPosition(), 1000, outtakeState);
            robot.intake.setState(robot.intakeMotor.getCurrentPosition(), 500,  intakeState);
            robot.droneLauncher.setState(robot.planeLauncher.getCurrentPosition(), 2000, droneLauncherState);

            //sequence 1
            robot.intake.wheelMotor.setPower(robot.intake.intakeToTicksParallelPowerPSequence(trueState, intakeState));
            robot.outtake.lsFront.setPower(robot.outtake.lsToTicksParallelPowerPSequence(intakeState, outtakeState, 1));
            robot.outtake.lsBack.setPower(robot.outtake.lsToTicksParallelPowerPSequence(intakeState, outtakeState, 1));

            //sequence 2
            robot.droneLauncher.wheel.setPower(robot.droneLauncher.wheelParallelPowerPSequence(trueState, droneLauncherState));

            telemetry.addData("intake ticks", intakeState.getError());
            telemetry.addData("outtake ticks", outtakeState.getError());
            telemetry.addData("drone ticks", droneLauncherState.getError());
            if (trueState.isDone() && !droneLauncherState.isDone() && opModeIsActive()) {
                telemetry.addLine("drone should be moving");
            }
            telemetry.update();

            if (intakeState.isDone() && outtakeState.isDone() && droneLauncherState.isDone()) {
                break;
            }
        }

//        robot.lsFront.setPower(0.001 * (outtakeState.getError()));
//        robot.intake.setPower(0.001*(intakeState.getError()));
//        robot.planeLauncher.setPower(0.001*(droneLauncherState.getError()));

    }
}
