package developing;

import util.CodeSeg;

public class RobotFunctions2 {
    private TestRobot bot;
    public void init(TestRobot t) {
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
}
