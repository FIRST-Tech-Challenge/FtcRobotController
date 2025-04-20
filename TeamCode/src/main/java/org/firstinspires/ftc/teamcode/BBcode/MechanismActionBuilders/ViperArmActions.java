package org.firstinspires.ftc.teamcode.BBcode.MechanismActionBuilders;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.SequentialAction;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.acmerobotics.roadrunner.Action;
import org.firstinspires.ftc.teamcode.BBcode.MechanismControllers.Arm;
import org.firstinspires.ftc.teamcode.BBcode.TelemetryHelper;
import org.firstinspires.ftc.teamcode.BBcode.MechanismControllers.Viper;
import org.firstinspires.ftc.teamcode.BBcode.UtilClasses.UtilActions;

public class ViperArmActions {
    //_Arm and _Viper must be global to allow use in the action classes
     Arm _Arm;
     Viper _Viper;
    WristClawActions _WristClawActions;

    //constructor
    public ViperArmActions(OpMode opMode) {
        _Arm = new Arm(opMode, new TelemetryHelper(opMode));
        _Viper = new Viper(opMode);
        _WristClawActions = new WristClawActions(opMode);
    }
    //--------------------------------------------------------------------------

    //Moves Arm to the HighBasket
    public class MoveArmToHighBasketAction implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet){
            _Arm.MoveToHighBasket();
            return false;
        }
    }
    public Action MoveArmToHighBasket() {
        return new MoveArmToHighBasketAction();
    }
    //Moves Viper to the HighBasket
    public class MoveViperToHighBasketAction implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            _Viper.ExtendFull(1);
            return false;
        }
    }
    public Action MoveViperToHighBasket() {
        return new MoveViperToHighBasketAction();
    }

    //Moves Arm to the Specimen
    public class MoveArmToSpecimenAction implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet){
            _Arm.MoveToSpecimen();
            return false;
        }
    }
    public Action MoveArmToSpecimen() {
        return new MoveArmToSpecimenAction();
    }
    //Moves Viper to the Specimen
    public class MoveViperToSpecimenAction implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            _Viper.ExtendSpecimenhang(1);
            return false;
        }
    }
    public Action MoveViperToSpecimen() {
        return new MoveViperToSpecimenAction();
    }

    //Moves Arm to Home
    public class MoveArmToHomeAction implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            _Arm.MoveToHome();
            return false;
        }
    }
    public Action MoveArmToHome() {
        return new MoveArmToHomeAction();
    }

    //Moves Arm to SlowDown
    public class MoveArmToSlowDownAction implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            _Arm.MoveToSlowDown();
            return false;
        }
    }
    public Action MoveArmToSlowDown() {
        return new MoveArmToSlowDownAction();
    }

    //Moves Viper to Home
    public class MoveViperToHomeAction implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            _Viper.ExtendClosed(1);
            return false;
        }
    }
    public Action MoveViperToHome() {
        return new MoveViperToHomeAction();
    }

    public class MoveViperHalfExtendAction implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            _Viper.ExtendHalf(1);
            return false;
        }
    }
    public Action MoveViperHalfExtend() {
        return new MoveViperHalfExtendAction();
    }

    //Moves Viper to pickup sample
    public class MoveViperToLongSamplePickUpAction implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            _Viper.Extendlongsubmersible(1);
            return false;
        }
    }
    public Action MoveViperToLongSamplePickUp() {
        return new MoveViperToLongSamplePickUpAction();
    }

    public class MoveViperToShortSamplePickUpAction implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            _Viper.Extendshortsubmersible(1);
            return false;
        }
    }
    public Action MoveViperToShortSamplePickUp() {
        return new MoveViperToShortSamplePickUpAction();
    }

    //dump in high basket
    public Action DumpInHighBasket() {
        return new SequentialAction(
                MoveArmToHighBasket(),
                _WristClawActions.WristDown(),
                UtilActions.Wait(1),
                MoveViperToHighBasket(),
                UtilActions.Wait(.5),
                _WristClawActions.WristUp(),
                UtilActions.Wait(.5),
                _WristClawActions.OpenClaw(),
                UtilActions.Wait(0.1),
                _WristClawActions.WristDown(),
                UtilActions.Wait(0.2),
                MoveViperToHome(),
                UtilActions.Wait(0.75),
                MoveArmToSlowDown(),
                UtilActions.Wait(0.75),
                MoveArmToHome(),
                UtilActions.Wait(0.25)
        );
    }

    public Action DumpInHighBasketHalfExtend() {
        return new SequentialAction(
                MoveArmToHighBasket(),
                _WristClawActions.WristDown(),
                UtilActions.Wait(1),
                MoveViperToHighBasket(),
                UtilActions.Wait(.5),
                _WristClawActions.WristUp(),
                UtilActions.Wait(.5),
                _WristClawActions.OpenClaw(),
                UtilActions.Wait(0.1),
                _WristClawActions.WristDown(),
                UtilActions.Wait(0.2),
                MoveViperHalfExtend(),
                UtilActions.Wait(0.5),
                MoveArmToSlowDown(),
                UtilActions.Wait(0.75),
                MoveArmToHome(),
                UtilActions.Wait(0.25)
        );
    }

    public Action RaiseToClip() {
        return new SequentialAction(
            _WristClawActions.WristClip(),
            UtilActions.Wait(0.2),
            MoveArmToSpecimen(),
//            UtilActions.Wait(0.75),
            MoveViperToSpecimen(),
            UtilActions.Wait(0.75)
        );
    }
    public Action LowerFromClip() {
        return new SequentialAction(
                _WristClawActions.OpenClaw(),
                UtilActions.Wait(0.2),
                _WristClawActions.WristDown(),
                UtilActions.Wait(0.2),
                MoveViperToHome(),
                UtilActions.Wait(0.75),
                MoveArmToSlowDown(),
                UtilActions.Wait(0.3),
                MoveArmToHome(),
                UtilActions.Wait(0.25),
                _WristClawActions.WristSpecimenPickup()
        );
    }
}
