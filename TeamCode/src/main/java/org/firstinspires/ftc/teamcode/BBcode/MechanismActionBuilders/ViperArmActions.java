package org.firstinspires.ftc.teamcode.BBcode.MechanismActionBuilders;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.acmerobotics.roadrunner.Action;
import org.firstinspires.ftc.teamcode.BBcode.MechanismControllers.Arm;
import org.firstinspires.ftc.teamcode.BBcode.TelemetryHelper;
import org.firstinspires.ftc.teamcode.BBcode.MechanismControllers.Viper;

public class ViperArmActions {
    //_Arm and _Viper must be global to allow use in the action classes
     Arm _Arm;
     Viper _Viper;

    //constructor
    public ViperArmActions(OpMode opMode) {
        _Arm = new Arm(opMode, new TelemetryHelper(opMode));
        _Viper = new Viper(opMode);
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
    //Moves Viper to Home
    public class MoveViperToHomeAction implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            _Viper.ExtendShort(1);
            return false;
        }
    }
    public Action MoveViperToHome() {
        return new MoveViperToHomeAction();
    }
}
