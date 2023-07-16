package org.firstinspires.ftc.teamcode.Components.RFModules.Devices;

import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.op;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;

public class LEDStrip{
    RevBlinkinLedDriver blinkin;
    RevBlinkinLedDriver.BlinkinPattern[] stackLevelColors = {RevBlinkinLedDriver.BlinkinPattern.VIOLET, RevBlinkinLedDriver.BlinkinPattern.BLUE,
            RevBlinkinLedDriver.BlinkinPattern.YELLOW, RevBlinkinLedDriver.BlinkinPattern.ORANGE};

    public LEDStrip(){
        blinkin = op.hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");
//        blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
    }

    public void setStackLevelColor(int level) {
        blinkin.setPattern(stackLevelColors[level]);
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
    public void pattern29(){blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_PARTY_PALETTE);}





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