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

    //Generates Action for MoveToHighBasket
    public class MoveToHighBasketAction implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            _Arm.MoveToHighBasket();
            _Viper.ExtendFull(1);
            return false;
        }
    }
    public Action MoveToHighBasket() {
        return new MoveToHighBasketAction();
    }

    //Generates Action for MoveToHome
    public class MoveToHomeAction implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            _Arm.MoveToHome();
            _Viper.ExtendShort(1);
            return false;
        }
    }
    public Action MoveToHome() {
        return new MoveToHomeAction();
    }
}
