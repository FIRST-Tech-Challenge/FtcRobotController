package org.nknsd.teamcode.components.drivers;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.nknsd.teamcode.components.handlers.BallLiftHandler;
import org.nknsd.teamcode.components.handlers.LowerLiftState;
import org.nknsd.teamcode.components.handlers.RaiseLiftState;
import org.nknsd.teamcode.components.handlers.gamepad.GamePadHandler;
import org.nknsd.teamcode.components.utility.StateMachine;
import org.nknsd.teamcode.controlSchemes.defaults.BallLauncherControlScheme;
import org.nknsd.teamcode.frameworks.NKNComponent;

public class BallLiftDriver implements NKNComponent {
    GamePadHandler gamePadHandler;
    BallLiftHandler ballLiftHandler;
    BallLauncherControlScheme controlScheme;

    Runnable raiseLift = new Runnable() {
        @Override
        public void run() {
            StateMachine.INSTANCE.startAnonymous(new RaiseLiftState(ballLiftHandler));
        }
    };

    Runnable lowerLift = new Runnable() {
        @Override
        public void run() {
            StateMachine.INSTANCE.startAnonymous(new LowerLiftState(ballLiftHandler));
        }
    };


    //testCode below, do not activate unless you know what you are doing.
//    Runnable liftRaiseIncrease = new Runnable() {
//        @Override
//        public void run() {
//            ballLiftHandler.liftPosInc();
//        }
//    };
//    Runnable liftRaiseDecrease = new Runnable() {
//        @Override
//        public void run() {
//            ballLiftHandler.liftPosDec();
//        }
//    };

//    Runnable liftLowerIncrease = new Runnable() {
//        @Override
//        public void run() {
//            ballLiftHandler.botPosInc();
//        }
//    };
//
//    Runnable liftLowerDecrease = new Runnable() {
//        @Override
//        public void run() {
//            ballLiftHandler.botPosDec();
//        }
//    };

    @Override
    public boolean init(Telemetry telemetry, HardwareMap hardwareMap, Gamepad gamepad1, Gamepad gamepad2) {
        return true;
    }

    @Override
    public void init_loop(ElapsedTime runtime, Telemetry telemetry) {

    }

    @Override
    public void start(ElapsedTime runtime, Telemetry telemetry) {
        gamePadHandler.addListener(controlScheme.raiseLift(), raiseLift, "raiseLift");
        gamePadHandler.addListener(controlScheme.lowerLift(), lowerLift, "lowerLift");
//        gamePadHandler.addListener(controlScheme.liftUpInc(), liftRaiseIncrease, "liftMaxIncrease");
//        gamePadHandler.addListener(controlScheme.liftUpDec(), liftRaiseDecrease, "liftMaxDecrease");
//        gamePadHandler.addListener(controlScheme.liftDownInc(), liftLowerIncrease, "liftMinIncrease");
//        gamePadHandler.addListener(controlScheme.liftDownDec(), liftLowerDecrease, "liftMinIncrease");

    }

    @Override
    public void stop(ElapsedTime runtime, Telemetry telemetry) {

    }

    @Override
    public String getName() {
        return "";
    }

    @Override
    public void loop(ElapsedTime runtime, Telemetry telemetry) {

    }

    @Override
    public void doTelemetry(Telemetry telemetry) {

    }

    public void link(GamePadHandler gamePadHandler, BallLauncherControlScheme ballLauncherControlScheme, BallLiftHandler ballLiftHandler){
        this.gamePadHandler = gamePadHandler;
        this.controlScheme = ballLauncherControlScheme;
        this.ballLiftHandler = ballLiftHandler;
    }
}
