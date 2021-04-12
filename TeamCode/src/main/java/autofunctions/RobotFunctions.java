package autofunctions;

import global.TerraBot;
import globalfunctions.Constants;
import util.CodeSeg;

public class RobotFunctions {
    private TerraBot bot;
    public void init(TerraBot t) {
        bot = t;
    }
    public CodeSeg intake(final double pow) {
        return new CodeSeg() {
            @Override
            public void run() {
                bot.intake(pow);
            }
        };
    }
    public CodeSeg shootRF(int rings) {
        return RobotFunctionsHandler.combineSegs(new CodeSeg[]{
                intake(0), readyShooter(), shootIntoGoal(rings), stopOuttake()
        });
    }
    public CodeSeg stopOuttake() {
        return new CodeSeg() {
            @Override
            public void run() {
                bot.outtaking = false;
            }
        };
    }
    public CodeSeg turnArm (final double pos) {
        return new CodeSeg() {
            @Override
            public void run() {
                bot.startWobbleGoalWithEncoders(pos, 1);
            }
        };
    }
    public CodeSeg stopArm () {
        return new CodeSeg() {
            @Override
            public void run() {
                bot.stopWobbleGoal();
            }
        };
    }
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
    public CodeSeg moveWgTo(final double deg) {
        return new CodeSeg() {
            @Override
            public void run() {
                bot.moveArmWithEnc(deg, 1);
            }
        };
    }
    public CodeSeg resetHeadingUsingGyro(){
        return new CodeSeg() {
            @Override
            public void run() {
                bot.resetHeadingUsingGyro();
            }
        };
    }

    public CodeSeg resetPosUsingDisSensors(){
        return new CodeSeg() {
            @Override
            public void run() {
                bot.resetPosUsingDisSensors();
            }
        };
    }
    public CodeSeg resetAll(){
        return new CodeSeg() {
            @Override
            public void run() {
                bot.resetHeadingUsingGyro();
                bot.resetPosUsingDisSensors();
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
                bot.rp.setPosition(bot.pushControl.getPos(1));
                pause(0.4);
                bot.rp.setPosition(0.2);
                pause(0.4);
                bot.rp.setPosition(bot.pushControl.getPos(1));
            }
        };
    }

    public CodeSeg shootIntoGoal(final int numRings){
        return new CodeSeg() {
            @Override
            public void run() {
                while (!bot.autoAimer.hasReached){}
                pause(0.1);
                for (int i = 0; i < numRings; i++) {
                    bot.rp.setPosition(bot.pushControl.getPos(2));
                    pause(0.3);
                    bot.rp.setPosition(0.2);
                    pause(0.3);

                    if(i == 0){
                        bot.autoAimer.debug = 30;
                    }else{
                        bot.autoAimer.debug = 0;
                    }
                    bot.autoAimer.updateTargetSpeed();
                }
                pause(0.1);

                bot.rp.setPosition(bot.pushControl.getPos(0));
                bot.rh.setPower(0);
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




}
