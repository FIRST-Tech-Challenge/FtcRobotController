package telefunctions;

import com.qualcomm.robotcore.util.ElapsedTime;

//Used to make sure things arent clicked twice in multiple loop updates
public class ButtonController {
    public ElapsedTime timer = new ElapsedTime();
    public double waitTime = 0.3;
    public double lastTime = 0;
    //Checks if the boolean b has only been true once in the last waitTime seconds
    public boolean isPressedOnce(boolean in) {
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
