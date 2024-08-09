package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.NewStuff.DroneLauncher;
import org.firstinspires.ftc.teamcode.NewStuff.Intake;
import org.firstinspires.ftc.teamcode.NewStuff.Outtake;
import org.firstinspires.ftc.teamcode.NewStuff.TrueState;

@TeleOp
public class TestTeleOpp extends LinearOpMode {

    Intake intake;
    Outtake outtake;
    DroneLauncher droneLauncher;

    TrueState trueState;

    @Override
    public void runOpMode() throws InterruptedException {

        outtake.state.setOpMode(this);
        intake.state.setOpMode(this);
        droneLauncher.state.setOpMode(this);

        waitForStart();

        intake.wheelMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake.wheelMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        outtake.lsFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        outtake.lsFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        while (opModeIsActive()) {

            //setting current and target (current position, target position, enter a state)
            outtake.setState(outtake.lsFront.getCurrentPosition(), 1000);
            intake.setState(intake.wheelMotor.getCurrentPosition(), 500);
            droneLauncher.setState(droneLauncher.wheel.getCurrentPosition(), 2000);

            // robot.outtake.update
            //sequence 1
            intake.wheelMotor.setPower(intake.intakeToTicksCalcPowerParallelSequence(trueState));
            outtake.lsMove(outtake.lsToTicksCalcPowerParallelSequence(intake.state, 1));
//            outtake.lsFront.setPower(outtake.lsToTicksCalcPowerParallelSequence(intakeState, outtakeState, 1));
//            outtake.lsBack.setPower(outtake.lsToTicksCalcPowerParallelSequence(intakeState, outtakeState, 1));

            //sequence 2
            droneLauncher.wheel.setPower(droneLauncher.wheelCalcPowerParallelSequence(trueState));

            telemetry.addData("intake ticks", intake.state.getError());
            telemetry.addData("outtake ticks", outtake.state.getError());
            telemetry.addData("drone ticks", droneLauncher.state.getError());
            telemetry.update();

            if (intake.state.isDone() && outtake.state.isDone() && droneLauncher.state.isDone()) {
                break;
            }
        }

//        robot.lsFront.setPower(0.001 * (outtakeState.getError()));
//        robot.intake.setPower(0.001*(intakeState.getError()));
//        robot.planeLauncher.setPower(0.001*(droneLauncherState.getError()));

    }
}
