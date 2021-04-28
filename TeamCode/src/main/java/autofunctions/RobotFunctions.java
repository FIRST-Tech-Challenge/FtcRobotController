package autofunctions;

import global.TerraBot;
import globalfunctions.Constants;
import util.CodeSeg;

public class RobotFunctions {
    private TerraBot bot;
    public void init(TerraBot t) {
        bot = t;
    }
//    public CodeSeg intake(final double pow) {
//        return new CodeSeg() {
//            @Override
//            public void run() {
//                bot.intake(pow);
//            }
//        };
//    }
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
        return new CodeSeg() {
            @Override
            public void run() {
                bot.setClawPos(1);
            }
        };
    }
    public CodeSeg openClaw() {
        return new CodeSeg() {
            @Override
            public void run() {
                bot.setClawPos(0);
            }
        };
    }
    public CodeSeg claw(final int idx) {
        return new CodeSeg() {
            @Override
            public void run() {
                bot.claw(bot.cllControl.getPos(idx), bot.clrControl.getPos(idx));
            }
        };
    }

    public CodeSeg claw(final int idx, final double offset) {
        return new CodeSeg() {
            @Override
            public void run() {
                bot.claw(bot.cllControl.getPos(idx)+offset, bot.clrControl.getPos(idx)-offset);
            }
        };
    }

    public CodeSeg moveWgTo(final double deg) {
        return new CodeSeg() {
            @Override
            public void run() {
                bot.moveArmWithEncWithoutWGE(deg, 1);
                bot.moveArm(0);
            }
        };
    }

    public CodeSeg controlWGE(final double pos) {
        return new CodeSeg() {
            @Override
            public void run() {
                while (!bot.isControlWgeDone(pos)) {
                    bot.controlWGE(pos);
                    bot.moveArm(bot.getRestPowArm());
                }
                bot.wge.setPower(0);
                bot.moveArm(0);
            }
        };
    }



    public CodeSeg resetHeadingUsingGyro(){
        return new CodeSeg() {
            @Override
            public void run() {
                bot.updateOdoWithGyroAndCheck();
            }
        };
    }

    public CodeSeg resetPosUsingDisSensors(){
        return new CodeSeg() {
            @Override
            public void run() {
                bot.updateOdoWithLocalizerAndCheck();
            }
        };
    }
    public CodeSeg resetAll(){
        return new CodeSeg() {
            @Override
            public void run() {
                bot.updateOdoWithGyroAndCheck();
                bot.updateOdoWithLocalizerAndCheck();
            }
        };
    }

    public CodeSeg setShotMode(final int i){
        return new CodeSeg() {
            @Override
            public void run() {
                bot.autoAimer.shotMode = i;
            }
        };
    }
    public CodeSeg nextShotMode(){
        return new CodeSeg() {
            @Override
            public void run() {
                bot.autoAimer.nextShotMode();
            }
        };
    }

    public CodeSeg readyShooter(){
        return new CodeSeg() {
            @Override
            public void run() {
                while (!bot.autoAimer.hasPosBeenUpdated()) {}
                bot.outtaking = true;
//                bot.rh2.setPower(-1);
//                bot.rp.setPosition(bot.pushControl.getPos(1));
//                bot.rh2.setPower(0);
            }
        };
    }

    public CodeSeg readyShooterWithoutPush(){
        return new CodeSeg() {
            @Override
            public void run() {
                while (!bot.autoAimer.hasPosBeenUpdated()) {}
                bot.outtaking = true;
            }
        };
    }


    public CodeSeg shootRF(final int numRings){
        return new CodeSeg() {
            @Override
            public void run() {
                bot.intake(0);
                while (!bot.autoAimer.hasPosBeenUpdated()){}
                bot.outtaking = true;
                bot.autoAimer.shotMode = 0;
                while (!bot.autoAimer.hasReached) {}
                pause(0.7);
                bot.rs.setPower(Constants.RS_POW);
                pause(((double)numRings)/3);
                bot.outr.setPower(0);
                bot.outl.setPower(0);
                bot.rs.setPower(0);
                bot.outtaking = false;
                bot.autoAimer.done();
            }
        };
    }
    public CodeSeg pauseRfs(final double secs){
        return new CodeSeg() {
            @Override
            public void run() {
                pause(secs);
            }
        };
    }

    private void pause(final double secs){
//        timer.reset();
//        while (timer.seconds() < secs){}
        try { Thread.sleep((long)(secs * 1000)); } catch (InterruptedException ignore) {}
    }

    public CodeSeg overrideShooter(final boolean val){
        return new CodeSeg() {
            @Override
            public void run() {
                bot.autoAimer.override = val;
            }
        };
    }

    public CodeSeg saveForTele(){
        return new CodeSeg() {
            @Override
            public void run() {
                bot.saveForTele();
            }
        };
    }

//    public CodeSeg

}
