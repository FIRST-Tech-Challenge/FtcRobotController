package developing;

import com.qualcomm.robotcore.util.ElapsedTime;

public class ButtonController {
    public ElapsedTime timer = new ElapsedTime();
    public double waitTime = 0.3;
    public double lastTime = 0;
    public boolean isPressing(boolean in) {
        if (in) {
            if (timer.seconds() - lastTime >= waitTime) {
                lastTime = timer.seconds();
                return true;
            }
            lastTime = timer.seconds();
        }
        return false;
    }
}
