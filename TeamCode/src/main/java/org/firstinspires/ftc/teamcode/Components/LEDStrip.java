package org.firstinspires.ftc.teamcode.Components;

import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.op;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;


public class LEDStrip{
    RevBlinkinLedDriver blinkin;

    public void init(){
        blinkin = op.hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");
        blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
    }

    public void loop() {
//        basic colors

            red();

            op.sleep(5000);

            blue();

            op.sleep(5000);

            orange();

            op.sleep(5000);

            yellow();

            op.sleep(5000);

            gold();

            op.sleep(5000);

            white();

            op.sleep(5000);

            gray();

            op.sleep(5000);

            pink();

            op.sleep(5000);

            aqua();

            op.sleep(5000);

            green();

            op.sleep(5000);

            black();

            op.sleep(5000);

            confetti();

            op.sleep(5000);

            lime();

            op.sleep(5000);

            violet();

            op.sleep(5000);





            //compound colors

            bluegreen();

            op.sleep(5000);

            blueviolet();

            op.sleep(5000);

            lawngreen();

            op.sleep(5000);

            redorange();

            op.sleep(5000);

            skyblue();

            op.sleep(5000);



            //dark

            darkblue();

            op.sleep(5000);

            darkgray();

            op.sleep(5000);

            darkgreen();

            op.sleep(5000);

            darkred();

            op.sleep(5000);





            //breath

            breathblue();

            op.sleep(5000);

            breathgray();

            op.sleep(5000);

            breathred();

            op.sleep(5000);





            //bpm

            bpmforest();

            op.sleep(5000);

            bpmlava();

            op.sleep(5000);

            bpmocean();

            op.sleep(5000);

            bpmparty();

            op.sleep(5000);

            bpmrainbow();

            op.sleep(5000);





            //waves

            wavesforest();

            op.sleep(5000);

            waveslava();

            op.sleep(5000);

            wavesocean();

            op.sleep(5000);

            wavesparty();

            op.sleep(5000);

            wavesrainbow();

            op.sleep(5000);





            //cp

            cpbpm();

            op.sleep(5000);





            //cp1

            cp1breathfast();

            op.sleep(5000);

            cp1breathslow();

            op.sleep(5000);

            cp1endtoendblacktoblack();

            op.sleep(5000);

            cp1heartbeatfast();

            op.sleep(5000);

            cp1heartbeatmedium();

            op.sleep(5000);

            cp1hearbeatslow();

            op.sleep(5000);

            cp1larsonscanner();

            op.sleep(5000);

            cp1lightchase();

            op.sleep(5000);

            cp1shot();

            op.sleep(5000);

            cp1strobe();

            op.sleep(5000);





            //cp12

            cp12colorgradient();

            op.sleep(5000);

            cp12colorwaves();

            op.sleep(5000);

            cp12endtoendblend();

            op.sleep(5000);

            cp12sinelon();

            op.sleep(5000);

            cp12endtoendblend1to2();

            op.sleep(5000);

            cp12noblending();

            op.sleep(5000);

            cp12sparkle1on2();

            op.sleep(5000);

            cp12sparkle2on1();

            op.sleep(5000);

            cp12twinkles();

            op.sleep(5000);





            //cp2

            cp2breathfast();

            op.sleep(5000);

            cp2breathslow();

            op.sleep(5000);

            cp2endtoendblendtoblack();

            op.sleep(5000);

            cp2heartbeatfast();

            op.sleep(5000);

            cp2hearbeatmedium();

            op.sleep(5000);

            cp2heartbeatslow();

            op.sleep(5000);

            cp2larsonscanner();

            op.sleep(5000);

            cp2lightchase();

            op.sleep(5000);

            cp2();

            op.sleep(5000);

            cp2strobe();

            op.sleep(5000);





            //fire

            fire();

            op.sleep(5000);

            firelarge();

            op.sleep(5000);

            firemedium();

            op.sleep(5000);





            //heartbeat

            heartbeatblue();

            op.sleep(5000);

            heartbeatgray();

            op.sleep(5000);

            heartbeatred();

            op.sleep(5000);

            heartbeatwhite();

            op.sleep(5000);





            //larson scanner

            larsonscannergray();

            op.sleep(5000);

            larsonscannerred();

            op.sleep(5000);





            //light chase

            lightchaseblue();

            op.sleep(5000);

            lightchasegray();

            op.sleep(5000);

            lightchasered();

            op.sleep(5000);





            //rainbow

            rainbow();

            op.sleep(5000);

            rainbowforest();

            op.sleep(5000);

            rainbowrainbow();

            op.sleep(5000);

            rainbowlava();

            op.sleep(5000);

            rainbowocean();

            op.sleep(5000);

            rainbowparty();

            op.sleep(5000);

            rainbowwithglitter();

            op.sleep(5000);





            //shot

            shotblue();

            op.sleep(5000);

            shotred();

            op.sleep(5000);

            shotwhite();

            op.sleep(5000);





            //sinelon

            sinelonforest();

            op.sleep(5000);

            sinelonlava();

            op.sleep(5000);

            sinelonocean();

            op.sleep(5000);

            sinelonparty();

            op.sleep(5000);

            sinelonrainbow();

            op.sleep(5000);





            //strobe

            strobeblue();

            op.sleep(5000);

            strobegold();

            op.sleep(5000);

            strobered();

            op.sleep(5000);

            strobewhite();

            op.sleep(5000);





            //twinkles

            twinklesforest();

            op.sleep(5000);

            twinkleslava();

            op.sleep(5000);

            twinklesocean();

            op.sleep(5000);

            twinklesparty();

            op.sleep(5000);

            twinklesrainbow();

            op.sleep(5000);
    }





    //basic colors

    public void red(){
        blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
    }

    public void blue(){
        blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
    }

    public void orange(){
        blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.ORANGE);
    }

    public void yellow(){
        blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW);
    }

    public void gold(){
        blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.GOLD);
    }

    public void white(){
        blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.WHITE);
    }

    public void gray(){
        blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.GRAY);
    }

    public void pink (){
        blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.HOT_PINK);
    }

    public void aqua() {
        blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.AQUA);
    }

    public void green() {
        blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
    }

    public void black() {
        blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);
    }

    public void confetti() {
        blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.CONFETTI);
    }

    public void lime() {
        blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.LIME);
    }

    public void violet() {
        blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.VIOLET);
    }





    //compound colors

    public void bluegreen() {
        blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE_GREEN);
    }

    public void blueviolet() {
        blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE_VIOLET);
    }

    public void lawngreen() {
        blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.LAWN_GREEN);
    }

    public void redorange() {
        blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED_ORANGE);
    }

    public void skyblue() {
        blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.SKY_BLUE);
    }



    //dark

    public void darkblue() {
        blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.DARK_BLUE);
    }

    public void darkgray() {
        blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.DARK_GRAY);
    }

    public void darkgreen() {
        blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.DARK_GREEN);
    }

    public void darkred() {
        blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.DARK_RED);
    }





    //breath

    public void breathblue() {
        blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.BREATH_BLUE);
    }

    public void breathgray() {
        blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.BREATH_GRAY);
    }

    public void breathred() {
        blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.BREATH_RED);
    }





    //bpm

    public void bpmforest() {
        blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.BEATS_PER_MINUTE_FOREST_PALETTE);
    }

    public void bpmlava() {
        blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.BEATS_PER_MINUTE_LAVA_PALETTE);
    }

    public void bpmocean() {
        blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.BEATS_PER_MINUTE_OCEAN_PALETTE);
    }

    public void bpmparty() {
        blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.BEATS_PER_MINUTE_PARTY_PALETTE);
    }

    public void bpmrainbow() {
        blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.BEATS_PER_MINUTE_RAINBOW_PALETTE);
    }





    //waves

    public void wavesforest() {
        blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_FOREST_PALETTE);
    }

    public void waveslava() {
        blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_LAVA_PALETTE);
    }

    public void wavesocean() {
        blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_OCEAN_PALETTE);
    }

    public void wavesparty() {
        blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_PARTY_PALETTE);
    }

    public void wavesrainbow() {
        blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_RAINBOW_PALETTE);
    }





    //cp

    public void cpbpm() {
        blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP1_2_BEATS_PER_MINUTE);
    }





    //cp1

    public void cp1breathfast() {
        blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP1_BREATH_FAST);
    }

    public void cp1breathslow() {
        blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP1_BREATH_SLOW);
    }

    public void cp1endtoendblacktoblack() {
        blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP1_END_TO_END_BLEND_TO_BLACK);
    }

    public void cp1heartbeatfast() {
        blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP1_HEARTBEAT_FAST);
    }

    public void cp1heartbeatmedium() {
        blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP1_HEARTBEAT_MEDIUM);
    }

    public void cp1hearbeatslow() {
        blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP1_HEARTBEAT_SLOW);
    }

    public void cp1larsonscanner() {
        blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP1_LARSON_SCANNER);
    }

    public void cp1lightchase() {
        blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP1_LIGHT_CHASE);
    }

    public void cp1shot() {
        blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP1_SHOT);
    }

    public void cp1strobe() {
        blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP1_STROBE);
    }





    //cp12

    public void cp12colorgradient() {
        blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP1_2_COLOR_GRADIENT);
    }

    public void cp12colorwaves() {
        blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP1_2_COLOR_WAVES);
    }

    public void cp12endtoendblend() {
        blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP1_2_END_TO_END_BLEND);
    }

    public void cp12sinelon() {
        blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP1_2_SINELON);
    }

    public void cp12endtoendblend1to2() {
        blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP1_2_END_TO_END_BLEND_1_TO_2);
    }

    public void cp12noblending() {
        blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP1_2_NO_BLENDING);
    }

    public void cp12sparkle1on2() {
        blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP1_2_SPARKLE_1_ON_2);
    }

    public void cp12sparkle2on1() {
        blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP1_2_SPARKLE_2_ON_1);
    }

    public void cp12twinkles() {
        blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP1_2_TWINKLES);
    }





    //cp2

    public void cp2breathfast() {
        blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP2_BREATH_FAST);
    }

    public void cp2breathslow() {
        blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP2_BREATH_SLOW);
    }

    public void cp2endtoendblendtoblack() {
        blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP2_END_TO_END_BLEND_TO_BLACK);
    }

    public void cp2heartbeatfast() {
        blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP2_HEARTBEAT_FAST);
    }

    public void cp2hearbeatmedium() {
        blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP2_HEARTBEAT_MEDIUM);
    }

    public void cp2heartbeatslow() {
        blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP2_HEARTBEAT_SLOW);
    }

    public void cp2larsonscanner() {
        blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP2_LARSON_SCANNER);
    }

    public void cp2lightchase() {
        blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP2_LIGHT_CHASE);
    }

    public void cp2() {
        blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP2_SHOT);
    }

    public void cp2strobe() {
        blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP2_STROBE);
    }





    //fire

    public void fire (){
        blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.FIRE_LARGE);
    }

    public void firelarge() {
        blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.FIRE_LARGE);
    }

    public void firemedium() {
        blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.FIRE_MEDIUM);
    }





    //heartbeat

    public void heartbeatblue() {
        blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_BLUE);
    }

    public void heartbeatgray() {
        blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_GRAY);
    }

    public void heartbeatred() {
        blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_RED);
    }

    public void heartbeatwhite() {
        blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_WHITE);
    }





    //larson scanner

    public void larsonscannergray() {
        blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.LARSON_SCANNER_GRAY);
    }

    public void larsonscannerred() {
        blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.LARSON_SCANNER_RED);
    }





    //light chase

    public void lightchaseblue() {
        blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.LIGHT_CHASE_BLUE);
    }

    public void lightchasegray() {
        blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.LIGHT_CHASE_GRAY);
    }

    public void lightchasered() {
        blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.LIGHT_CHASE_RED);
    }





    //rainbow

    public void rainbow(){
        blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_RAINBOW_PALETTE);
    }

    public void rainbowforest() {
        blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_FOREST_PALETTE);
    }

    public void rainbowrainbow() {
        blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_RAINBOW_PALETTE);
    }

    public void rainbowlava() {
        blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_LAVA_PALETTE);
    }

    public void rainbowocean() {
        blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_OCEAN_PALETTE);
    }

    public void rainbowparty() {
        blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_PARTY_PALETTE);
    }

    public void rainbowwithglitter() {
        blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_WITH_GLITTER);
    }





    //shot

    public void shotblue() {
        blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.SHOT_BLUE);
    }

    public void shotred() {
        blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.SHOT_RED);
    }

    public void shotwhite() {
        blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.SHOT_WHITE);
    }





    //sinelon

    public void sinelonforest() {
        blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.SINELON_FOREST_PALETTE);
    }

    public void sinelonlava() {
        blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.SINELON_LAVA_PALETTE);
    }

    public void sinelonocean() {
        blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.SINELON_OCEAN_PALETTE);
    }

    public void sinelonparty() {
        blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.SINELON_PARTY_PALETTE);
    }

    public void sinelonrainbow() {
        blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.SINELON_RAINBOW_PALETTE);
    }





    //strobe

    public void strobeblue() {
        blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.STROBE_BLUE);
    }

    public void strobegold() {
        blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.STROBE_GOLD);
    }

    public void strobered() {
        blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.STROBE_RED);
    }

    public void strobewhite() {
        blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.STROBE_WHITE);
    }





    //twinkles

    public void twinklesforest() {
        blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.TWINKLES_FOREST_PALETTE);
    }

    public void twinkleslava() {
        blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.TWINKLES_LAVA_PALETTE);
    }

    public void twinklesocean() {
        blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.TWINKLES_OCEAN_PALETTE);
    }

    public void twinklesparty() {
        blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.TWINKLES_PARTY_PALETTE);
    }

    public void twinklesrainbow() {
        blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.TWINKLES_RAINBOW_PALETTE);
    }
}