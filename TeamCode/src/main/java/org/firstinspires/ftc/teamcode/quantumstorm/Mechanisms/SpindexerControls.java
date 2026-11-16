package org.firstinspires.ftc.teamcode.quantumstorm.Mechanisms;

/** Routes the shared flower-arm/spindexer D-pad without feeding on modifier release. */
public final class SpindexerControls {
    private boolean flowerPress;

    /** Returns forward (1), reverse (-1), or stopped (0) demand; call every loop. */
    public int update(boolean up,boolean down,boolean back,boolean leftBumper) {
        if(!up && !down) {
            flowerPress=false;
            return 0;
        }
        if(back || leftBumper) {
            flowerPress=true;
        }
        // Once a press belongs to the flower arm, releasing only its modifier
        // must not turn it into a spindexer command. Both directions must clear.
        if(flowerPress)return 0;
        return up?1:-1;
    }
}
