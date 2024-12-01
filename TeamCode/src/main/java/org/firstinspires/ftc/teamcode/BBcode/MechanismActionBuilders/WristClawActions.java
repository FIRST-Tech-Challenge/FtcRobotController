package org.firstinspires.ftc.teamcode.BBcode.MechanismActionBuilders;

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


    //Generates Action for WristUp
    public class WristUpAction implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            _WristClaw.WristUp();
            return false;
        }
    }
    public Action WristUp() {
        return new WristUpAction();
    }


    //Generates Action for WristFlip
    public class WristFlipAction implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            _WristClaw.WristFlip();
            return false;
        }
    }
    public Action WristFlip() {
        return new WristFlipAction();
    }


    //Generates Action for WristDown
    public class WristDownAction implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            _WristClaw.WristDown();
            return false;
        }
    }
    public Action WristDown() {
        return new WristDownAction();
    }


    //Generates Action for WristDump
    public class WristDumpAction implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            _WristClaw.WristDump();
            return false;
        }
    }
    public Action WristDump() {
        return new WristDumpAction();
    }


    //Generates Action for WristCenter
    public class WristCenterAction implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            _WristClaw.WristCenter();
            return false;
        }
    }
    public Action WristCenter() {
        return new WristCenterAction();
    }


    //Generates Action for WristInit
    public class WristInitAction implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            _WristClaw.WristInit();
            return false;
        }
    }
    public Action WristInit() {
        return new WristInitAction();
    }
}
