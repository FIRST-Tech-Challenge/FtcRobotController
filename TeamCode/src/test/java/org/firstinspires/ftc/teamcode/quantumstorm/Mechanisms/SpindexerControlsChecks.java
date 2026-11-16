package org.firstinspires.ftc.teamcode.quantumstorm.Mechanisms;

import org.firstinspires.ftc.teamcode.biobuzz.FeedCadence;

/** Desktop regression checks for controller sequences; no FTC hardware needed. */
public final class SpindexerControlsChecks {
    private static void check(boolean condition,String why) {
        if(!condition)throw new AssertionError(why);
    }

    private static void releaseOrder(boolean up,boolean back) {
        SpindexerControls controls=new SpindexerControls();
        int expected=up?1:-1;
        check(controls.update(up,!up,back,!back)==0,"Flower combo must not feed");
        check(controls.update(up,!up,false,false)==0,"Modifier-first release must not feed");
        check(controls.update(up,!up,false,false)==0,"Held D-pad stays blocked");
        check(controls.update(false,false,false,false)==0,"Release clears reservation");
        check(controls.update(up,!up,false,false)==expected,"Fresh D-pad press works");

        controls=new SpindexerControls();
        controls.update(up,!up,back,!back);
        check(controls.update(false,false,back,!back)==0,"D-pad-first release stops");
        controls.update(false,false,false,false);
        check(controls.update(up,!up,false,false)==expected,"Either release order rearms");

        controls=new SpindexerControls();
        check(controls.update(up,!up,false,false)==expected,"Plain D-pad works initially");
        check(controls.update(up,!up,back,!back)==0,"Adding modifier takes over D-pad");
        check(controls.update(up,!up,false,false)==0,"Removing modifier does not resume feed");
    }

    public static void main(String[] args) {
        for(boolean up:new boolean[]{true,false})for(boolean back:new boolean[]{true,false})
            releaseOrder(up,back);

        SpindexerControls controls=new SpindexerControls();
        controls.update(true,false,true,true);
        check(controls.update(true,false,false,true)==0,"One modifier remains held");
        check(controls.update(true,false,false,false)==0,"Both modifiers released still blocks");
        check(controls.update(true,true,false,false)==0,"Overlapping directions stay blocked");
        check(controls.update(false,true,false,false)==0,"Rolling to opposite direction stays blocked");
        controls.update(false,false,false,false);
        check(controls.update(false,true,false,false)==-1,"Fresh reverse is immediate");

        controls=new SpindexerControls();
        controls.update(false,false,true,false);
        controls.update(false,false,false,false);
        check(controls.update(true,false,false,false)==1,"Modifier alone does not consume next press");

        // Confirm a consumed press cannot start a feed pulse even after a long wait,
        // and that taking over an active pulse preserves the normal cooldown.
        controls=new SpindexerControls();
        FeedCadence cadence=new FeedCadence();
        check(cadence.apply(controls.update(true,false,false,false),0)==1,"Normal forward pulse");
        check(cadence.apply(controls.update(true,false,true,false),.05)==0,"Flower combo stops pulse");
        check(cadence.apply(controls.update(true,false,false,false),2)==0,"No delayed accidental pulse");
        cadence.apply(controls.update(false,false,false,false),2.01);
        check(cadence.apply(controls.update(true,false,false,false),2.02)==1,"Fresh press can feed");
        cadence.apply(controls.update(true,false,false,true),2.03);
        cadence.apply(controls.update(false,false,false,false),2.04);
        check(cadence.apply(controls.update(true,false,false,false),2.05)==0,"Fresh press respects feed gap");
        System.out.println("Spindexer modifier release, fresh press, direction rollover and feed cadence checks passed.");
    }
}
