package autofunctions;

import global.TerraBot;
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
    public CodeSeg startOuttake(final TerraBot bot) {
        return new CodeSeg() {
            @Override
            public void run() {
                bot.outtaking = true;
                bot.resetOuttake();
            }
        };
    }
    public CodeSeg stopOuttake() {
        return new CodeSeg() {
            @Override
            public void run() {
                bot.outtaking = false;
            }
        };
    }

}
