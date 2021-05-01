package autofunctions;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import global.TerraBot;
import globalfunctions.Constants;
import globalfunctions.Sleep;
import util.CodeSeg;

public class RobotFunctions {
    private TerraBot bot;
    public void init(TerraBot t) {
        bot = t;
    }
    public CodeSeg intake(final double pow) {
        return () -> bot.intake(pow);
    }
//    public CodeSeg shootRF(int rings) {
//        return RobotFunctionsHandler.combineSegs(new CodeSeg[]{
//                shootIntoGoal(rings)
//        });
//    }
//    public CodeSeg stopOuttake() {
//        return new CodeSeg() {
//            @Override
//            public void run() {
//                bot.outtaking = false;
//            }
//        };
//    }
//    public CodeSeg stopArm () {
//        return new CodeSeg() {
//            @Override
//            public void run() {
//                bot.stopWobbleGoal();
//            }
//        };
//    }
    public CodeSeg wgMoveFront() {
        return moveWgTo(Constants.WG_LOWER_LIMIT);
    }
    public CodeSeg wgMoveBack() {
        return moveWgTo(Constants.WG_UPPER_LIMIT);
    }
    public CodeSeg closeClaw() {
        return () -> bot.setClawPos(1);
    }
    public CodeSeg openClaw() {
        return () -> bot.setClawPos(0);
    }
    public CodeSeg claw(final int idx) {
        return () -> bot.claw(bot.cllControl.getPos(idx), bot.clrControl.getPos(idx));
    }

    public CodeSeg claw(final int idx, final double offset) {
        return () -> bot.claw(bot.cllControl.getPos(idx)+offset, bot.clrControl.getPos(idx)-offset);
    }

    public CodeSeg moveWgTo(final double deg) {
        return () -> {
            bot.moveArmWithEncWithoutWGE(deg, 1);
            bot.moveArm(0);
        };
    }

    public CodeSeg controlWGE(final double pos) {
        return () -> {
            while (!bot.isControlWgeDone(pos)) {
                bot.controlWGE(pos);
                bot.moveArm(bot.getRestPowArm());
            }
            bot.wge.setPower(0);
            bot.moveArm(0);
        };
    }



    public CodeSeg resetHeadingUsingGyro(){
        return () -> bot.updateOdoWithGyroAndCheck();
    }

    public CodeSeg resetPosUsingDisSensors(){
        return () -> bot.updateOdoWithLocalizerAndCheck();
    }
    public CodeSeg resetAll(){
        return () -> {
            bot.updateOdoWithGyroAndCheck();
            bot.updateOdoWithLocalizerAndCheck();
        };
    }

    public CodeSeg setShotMode(final int i){
        return () -> bot.autoAimer.shotMode = i;
    }
    public CodeSeg nextShotMode(){
        return () -> bot.autoAimer.nextShotMode();
    }

    public CodeSeg readyShooter(){
        return () -> {
            while (!bot.autoAimer.hasPosBeenUpdated()) {}
            bot.outtaking = true;
//                bot.rh2.setPower(-1);
//                bot.rp.setPosition(bot.pushControl.getPos(1));
//                bot.rh2.setPower(0);
        };
    }

    public CodeSeg readyShooterWithoutPush(){
        return () -> {
            while (!bot.autoAimer.hasPosBeenUpdated()) {}
            bot.outtaking = true;
        };
    }


    public CodeSeg shootRF(final int numRings, final LinearOpMode op){
        return () -> {
            bot.intake(0);
            bot.outtaking = true;
            bot.autoAimer.shotMode = 0;
            op.telemetry.addData("Starting loop", "");
            op.telemetry.update();
            while (!bot.autoAimer.hasReached) {
                Sleep.trySleep(() -> Thread.sleep(50));
//                op.telemetry.addData("Reached T2", bot.autoAimer.hasReached);
            }
            op.telemetry.addData("Ending loop", "");
            op.telemetry.update();
            pause(0.7);
            bot.rs.setPower(Constants.RS_POW);
            pause(((double)numRings)/3);
            bot.outr.setPower(0);
            bot.outl.setPower(0);
            bot.rs.setPower(0);
            bot.outtaking = false;
            bot.autoAimer.done();
        };
    }
    public CodeSeg pauseRfs(final double secs){
        return () -> pause(secs);
    }

    private void pause(final double secs){
//        timer.reset();
//        while (timer.seconds() < secs){}
        Sleep.trySleep(() -> Thread.sleep((long)(secs * 1000)));
//        try { Thread.sleep((long)(secs * 1000)); } catch (InterruptedException ignore) {}
    }


    public CodeSeg saveForTele(){
        return () -> bot.saveForTele();
    }

}
