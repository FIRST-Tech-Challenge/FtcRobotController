package org.nknsd.teamcode.controlSchemes.defaults;

import org.nknsd.teamcode.components.handlers.gamepad.GamePadHandler;
import org.nknsd.teamcode.frameworks.NKNControlScheme;

import java.util.concurrent.Callable;

public class BallLauncherControlScheme extends NKNControlScheme {

    public String getName() { return "Default"; }

    boolean raiseLiftDelay = false;
    boolean lowerLiftDelay = false;

    public Callable<Boolean> raiseLift(){
        return new Callable<Boolean>() {
            @Override
            public Boolean call() throws Exception {
                boolean button = GamePadHandler.GamepadButtons.A.detect(gamePadHandler.getGamePad2());
                if(!raiseLiftDelay && button){
                    raiseLiftDelay = true;
                    return true;
                } else if (!button){
                    raiseLiftDelay = false;
                }

                return false;
            }
        };

    }
    public Callable<Boolean> lowerLift(){
        return new Callable<Boolean>() {
            @Override
            public Boolean call() throws Exception {
                boolean button = GamePadHandler.GamepadButtons.B.detect(gamePadHandler.getGamePad2());
                if(!lowerLiftDelay && button){
                    lowerLiftDelay = true;
                    return true;
                } else if (!button){
                    lowerLiftDelay = false;
                }

                return false;
            }
        };

    }

//    boolean liftUpIncDelay = false;
//    boolean liftUpDecDelay = false;
//    boolean liftDownIncDelay = false;
//    boolean liftDownDecDelay = false;
//    public Callable<Boolean> liftUpInc(){
//        return new Callable<Boolean>() {
//            @Override
//            boolean button = GamePadHandler.GamepadButtons.DPAD_UP.detect(gamePadHandler.getGamePad2());
//                if(!liftUpIncDelay && button){
//                liftUpIncDelay = true;
//                return true;
//            } else if (!button){
//                liftUpIncDelay = false;
//            }
//
//                return false;            }
//    };
//}
//
//public Callable<Boolean> liftUpDec(){
//    return new Callable<Boolean>() {
//        @Override
//        public Boolean call() throws Exception {
//            boolean button = GamePadHandler.GamepadButtons.DPAD_DOWN.detect(gamePadHandler.getGamePad2());
//            if(!liftUpDecDelay && button){
//                liftUpDecDelay = true;
//                return true;
//            } else if (!button){
//                liftUpDecDelay = false;
//            }
//
//            return false;            }
//    };
//}
//public Boolean call() throws Exception {
//
//    public Callable<Boolean> liftDownInc(){
//        return new Callable<Boolean>() {
//            @Override
//            public Boolean call() throws Exception {
//                boolean button = GamePadHandler.GamepadButtons.DPAD_LEFT.detect(gamePadHandler.getGamePad2());
//                if(!liftDownIncDelay && button){
//                    liftDownIncDelay = true;
//                    return true;
//                } else if (!button){
//                    liftDownIncDelay = false;
//                }
//
//                return false;            }
//        };
//    }
//
//    public Callable<Boolean> liftDownDec(){
//        return new Callable<Boolean>() {
//            @Override
//            public Boolean call() throws Exception {
//                boolean button = GamePadHandler.GamepadButtons.DPAD_RIGHT.detect(gamePadHandler.getGamePad2());
//                if(!liftDownDecDelay && button){
//                    liftDownDecDelay = true;
//                    return true;
//                } else if (!button){
//                    liftDownDecDelay = false;
//                }
//
//                return false;            }
//        };
//    }
}
