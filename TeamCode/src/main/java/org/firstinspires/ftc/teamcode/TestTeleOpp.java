package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.NewStuff.ActionSet;
import org.firstinspires.ftc.teamcode.NewStuff.DroneLauncher;
import org.firstinspires.ftc.teamcode.NewStuff.Intake;
import org.firstinspires.ftc.teamcode.NewStuff.MoveLSAction;
import org.firstinspires.ftc.teamcode.NewStuff.MoveTrayClampAction;
import org.firstinspires.ftc.teamcode.NewStuff.OpModeUtilities;
import org.firstinspires.ftc.teamcode.NewStuff.Outtake;
import org.firstinspires.ftc.teamcode.NewStuff.TurnDroneLauncherWheelAction;
import org.firstinspires.ftc.teamcode.NewStuff.TurnIntakeWheelAction;
import org.firstinspires.ftc.teamcode.NewStuff.WaitAction;

@TeleOp
public class TestTeleOpp extends LinearOpMode {

    Intake intake;
    Outtake outtake;
    DroneLauncher droneLauncher;

    OpModeUtilities opModeUtilities;


    @Override
    public void runOpMode() throws InterruptedException {

        opModeUtilities = new OpModeUtilities(hardwareMap, this, telemetry);

        outtake = new Outtake(opModeUtilities);
        intake = new Intake(opModeUtilities);
        droneLauncher = new DroneLauncher(opModeUtilities);

        //define the actions; enter a condition (if any), target ticks, and module
//        TurnIntakeWheelAction action1 = new TurnIntakeWheelAction(1000, intake);
//        MoveLSAction action2 = new MoveLSAction(action1,1000, outtake);
//        TurnIntakeWheelAction action3 = new TurnIntakeWheelAction(action2, 2000, intake);
//        MoveLSAction action4 = new MoveLSAction(action2, 2000, outtake);

        //build a chain of actions
        ActionSet outer = new ActionSet();

        ActionSet actions1 = new ActionSet();
        actions1.scheduleParallel(new MoveLSAction(500, outtake));
        actions1.scheduleParallel(new TurnIntakeWheelAction(500, intake));

        ActionSet actions2 = new ActionSet();
        actions2.scheduleSequential(new TurnDroneLauncherWheelAction(500, droneLauncher));
        actions2.scheduleSequential(new MoveTrayClampAction(0.5, outtake));
        actions2.scheduleSequential(new WaitAction(5));
        // outer.scheduleSequential(bundle);

        MoveLSAction action3 = new MoveLSAction(actions2, 0, outtake);

        outer.scheduleParallel(actions1);
        outer.scheduleParallel(actions2);
        outer.scheduleSequential(action3);

        waitForStart();

        while (opModeIsActive()) {

            //update actions, run when given condition is true
//            action1.update();
//            action2.update();
//            action3.update();
//            action4.update();

            //moveLS.updateCheckDone();
            outer.updateCheckDone();

            telemetry.addData("outtake ticks", outtake.lsFront.getCurrentPosition());
            telemetry.addData("drone launcher ticks", droneLauncher.wheel.getCurrentPosition());
            telemetry.update();

        }
    }
}
