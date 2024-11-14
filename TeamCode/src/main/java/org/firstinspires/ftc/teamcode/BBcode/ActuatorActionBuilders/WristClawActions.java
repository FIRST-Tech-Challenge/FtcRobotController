package org.firstinspires.ftc.teamcode.BBcode.ActuatorActionBuilders;

import androidx.annotation.NonNull;
import org.firstinspires.ftc.teamcode.BBcode.WristClaw;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.acmerobotics.roadrunner.Action;

public class WristClawActions {
    //WristClaw must be global to allow use in the action classes
    WristClaw _WristClaw;

    //constructor
    public WristClawActions(OpMode opMode) {
        _WristClaw = new WristClaw(opMode);
    }
    //--------------------------------------------------------------------------

    //Generates Action for OpenClaw
    public class OpenClawAction implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            _WristClaw.OpenClaw();
            return false;
        }
    }
    public Action OpenClaw() {
        return new OpenClawAction();
    }



    //Generates Action for CloseClaw
    public class CloseClaw implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            _WristClaw.CloseClaw();
            return false;
        }
    }

    //Generates Action for MoveUp
    public class MoveUp implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            _WristClaw.MoveUp();
            return false;
        }
    }

    //Generates Action for MoveFlip
    public class MoveFlip implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            _WristClaw.MoveFlip();
            return false;
        }
    }

    //Generates Action for MoveDown
    public class MoveDown implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            _WristClaw.MoveDown();
            return false;
        }
    }

    //Generates Action for MoveDump
    public class MoveDump implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            _WristClaw.MoveDump();
            return false;
        }
    }

    //Generates Action for MoveCenter
    public class MoveCenter implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            _WristClaw.MoveCenter();
            return false;
        }
    }

    //Generates Action for MoveWristInit
    public class MoveWristInit implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            _WristClaw.MoveWristInit();
            return false;
        }
    }
}
