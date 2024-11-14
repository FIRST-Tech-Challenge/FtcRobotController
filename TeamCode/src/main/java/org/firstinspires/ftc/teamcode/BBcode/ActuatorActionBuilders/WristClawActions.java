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
    public class CloseClawAction implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            _WristClaw.CloseClaw();
            return false;
        }
    }
    public Action CloseClaw() {
        return new CloseClawAction();
    }


    //Generates Action for MoveUp
    public class MoveUpAction implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            _WristClaw.MoveUp();
            return false;
        }
    }
    public Action MoveUp() {
        return new MoveUpAction();
    }


    //Generates Action for MoveFlip
    public class MoveFlipAction implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            _WristClaw.MoveFlip();
            return false;
        }
    }
    public Action MoveFlip() {
        return new MoveFlipAction();
    }


    //Generates Action for MoveDown
    public class MoveDownAction implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            _WristClaw.MoveDown();
            return false;
        }
    }
    public Action MoveDown() {
        return new MoveDownAction();
    }


    //Generates Action for MoveDump
    public class MoveDumpAction implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            _WristClaw.MoveDump();
            return false;
        }
    }
    public Action MoveDump() {
        return new MoveDumpAction();
    }


    //Generates Action for MoveCenter
    public class MoveCenterAction implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            _WristClaw.MoveCenter();
            return false;
        }
    }
    public Action MoveCenter() {
        return new MoveCenterAction();
    }


    //Generates Action for MoveWristInit
    public class MoveWristInitAction implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            _WristClaw.MoveWristInit();
            return false;
        }
    }
    public Action MoveWristInit() {
        return new MoveWristInitAction();
    }
}
