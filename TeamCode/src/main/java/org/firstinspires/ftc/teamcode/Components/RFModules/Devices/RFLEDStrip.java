package org.firstinspires.ftc.teamcode.Components.RFModules.Devices;

import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.op;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;

public class RFLEDStrip {
    //init blinkin, define conestack colors preset
    private RevBlinkinLedDriver blinkIn;
    private RevBlinkinLedDriver.BlinkinPattern[] stackLevelColors = {RevBlinkinLedDriver.BlinkinPattern.VIOLET, RevBlinkinLedDriver.BlinkinPattern.BLUE,
            RevBlinkinLedDriver.BlinkinPattern.YELLOW, RevBlinkinLedDriver.BlinkinPattern.ORANGE};
    //constructor
    public RFLEDStrip(){
        blinkIn = op.hardwareMap.get(RevBlinkinLedDriver.class, "blinkIn");
    }
    //change leds w/ passed in integer stack level
    public void setStackLevelColor(int level) {
        blinkIn.setPattern(stackLevelColors[level]);
    }



    //basic colors

    public void red(){
        blinkIn.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
    }

    public void blue(){
        blinkIn.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
    }

    public void orange(){
        blinkIn.setPattern(RevBlinkinLedDriver.BlinkinPattern.ORANGE);
    }

    public void yellow(){
        blinkIn.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW);
    }

    public void gold(){
        blinkIn.setPattern(RevBlinkinLedDriver.BlinkinPattern.GOLD);
    }

    public void white(){
        blinkIn.setPattern(RevBlinkinLedDriver.BlinkinPattern.WHITE);
    }

    public void gray(){
        blinkIn.setPattern(RevBlinkinLedDriver.BlinkinPattern.GRAY);
    }

    public void pink (){
        blinkIn.setPattern(RevBlinkinLedDriver.BlinkinPattern.HOT_PINK);
    }

    public void aqua() {
        blinkIn.setPattern(RevBlinkinLedDriver.BlinkinPattern.AQUA);
    }

    public void green() {
        blinkIn.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
    }

    public void black() {
        blinkIn.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);
    }

    public void confetti() {
        blinkIn.setPattern(RevBlinkinLedDriver.BlinkinPattern.CONFETTI);
    }

    public void lime() {
        blinkIn.setPattern(RevBlinkinLedDriver.BlinkinPattern.LIME);
    }

    public void violet() {
        blinkIn.setPattern(RevBlinkinLedDriver.BlinkinPattern.VIOLET);
    }
    public void pattern29(){blinkIn.setPattern(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_PARTY_PALETTE);}





    //compound colors

    public void bluegreen() {
        blinkIn.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE_GREEN);
    }

    public void blueviolet() {
        blinkIn.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE_VIOLET);
    }

    public void lawngreen() {
        blinkIn.setPattern(RevBlinkinLedDriver.BlinkinPattern.LAWN_GREEN);
    }

    public void redorange() {
        blinkIn.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED_ORANGE);
    }

    public void skyblue() {
        blinkIn.setPattern(RevBlinkinLedDriver.BlinkinPattern.SKY_BLUE);
    }



    //dark

    public void darkblue() {
        blinkIn.setPattern(RevBlinkinLedDriver.BlinkinPattern.DARK_BLUE);
    }

    public void darkgray() {
        blinkIn.setPattern(RevBlinkinLedDriver.BlinkinPattern.DARK_GRAY);
    }

    public void darkgreen() {
        blinkIn.setPattern(RevBlinkinLedDriver.BlinkinPattern.DARK_GREEN);
    }

    public void darkred() {
        blinkIn.setPattern(RevBlinkinLedDriver.BlinkinPattern.DARK_RED);
    }





    //breath

    public void breathblue() {
        blinkIn.setPattern(RevBlinkinLedDriver.BlinkinPattern.BREATH_BLUE);
    }

    public void breathgray() {
        blinkIn.setPattern(RevBlinkinLedDriver.BlinkinPattern.BREATH_GRAY);
    }

    public void breathred() {
        blinkIn.setPattern(RevBlinkinLedDriver.BlinkinPattern.BREATH_RED);
    }





    //bpm

    public void bpmforest() {
        blinkIn.setPattern(RevBlinkinLedDriver.BlinkinPattern.BEATS_PER_MINUTE_FOREST_PALETTE);
    }

    public void bpmlava() {
        blinkIn.setPattern(RevBlinkinLedDriver.BlinkinPattern.BEATS_PER_MINUTE_LAVA_PALETTE);
    }

    public void bpmocean() {
        blinkIn.setPattern(RevBlinkinLedDriver.BlinkinPattern.BEATS_PER_MINUTE_OCEAN_PALETTE);
    }

    public void bpmparty() {
        blinkIn.setPattern(RevBlinkinLedDriver.BlinkinPattern.BEATS_PER_MINUTE_PARTY_PALETTE);
    }

    public void bpmrainbow() {
        blinkIn.setPattern(RevBlinkinLedDriver.BlinkinPattern.BEATS_PER_MINUTE_RAINBOW_PALETTE);
    }





    //waves

    public void wavesforest() {
        blinkIn.setPattern(RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_FOREST_PALETTE);
    }

    public void waveslava() {
        blinkIn.setPattern(RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_LAVA_PALETTE);
    }

    public void wavesocean() {
        blinkIn.setPattern(RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_OCEAN_PALETTE);
    }

    public void wavesparty() {
        blinkIn.setPattern(RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_PARTY_PALETTE);
    }

    public void wavesrainbow() {
        blinkIn.setPattern(RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_RAINBOW_PALETTE);
    }





    //cp

    public void cpbpm() {
        blinkIn.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP1_2_BEATS_PER_MINUTE);
    }





    //cp1

    public void cp1breathfast() {
        blinkIn.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP1_BREATH_FAST);
    }

    public void cp1breathslow() {
        blinkIn.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP1_BREATH_SLOW);
    }

    public void cp1endtoendblacktoblack() {
        blinkIn.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP1_END_TO_END_BLEND_TO_BLACK);
    }

    public void cp1heartbeatfast() {
        blinkIn.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP1_HEARTBEAT_FAST);
    }

    public void cp1heartbeatmedium() {
        blinkIn.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP1_HEARTBEAT_MEDIUM);
    }

    public void cp1hearbeatslow() {
        blinkIn.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP1_HEARTBEAT_SLOW);
    }

    public void cp1larsonscanner() {
        blinkIn.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP1_LARSON_SCANNER);
    }

    public void cp1lightchase() {
        blinkIn.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP1_LIGHT_CHASE);
    }

    public void cp1shot() {
        blinkIn.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP1_SHOT);
    }

    public void cp1strobe() {
        blinkIn.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP1_STROBE);
    }





    //cp12

    public void cp12colorgradient() {
        blinkIn.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP1_2_COLOR_GRADIENT);
    }

    public void cp12colorwaves() {
        blinkIn.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP1_2_COLOR_WAVES);
    }

    public void cp12endtoendblend() {
        blinkIn.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP1_2_END_TO_END_BLEND);
    }

    public void cp12sinelon() {
        blinkIn.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP1_2_SINELON);
    }

    public void cp12endtoendblend1to2() {
        blinkIn.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP1_2_END_TO_END_BLEND_1_TO_2);
    }

    public void cp12noblending() {
        blinkIn.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP1_2_NO_BLENDING);
    }

    public void cp12sparkle1on2() {
        blinkIn.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP1_2_SPARKLE_1_ON_2);
    }

    public void cp12sparkle2on1() {
        blinkIn.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP1_2_SPARKLE_2_ON_1);
    }

    public void cp12twinkles() {
        blinkIn.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP1_2_TWINKLES);
    }





    //cp2

    public void cp2breathfast() {
        blinkIn.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP2_BREATH_FAST);
    }

    public void cp2breathslow() {
        blinkIn.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP2_BREATH_SLOW);
    }

    public void cp2endtoendblendtoblack() {
        blinkIn.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP2_END_TO_END_BLEND_TO_BLACK);
    }

    public void cp2heartbeatfast() {
        blinkIn.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP2_HEARTBEAT_FAST);
    }

    public void cp2hearbeatmedium() {
        blinkIn.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP2_HEARTBEAT_MEDIUM);
    }

    public void cp2heartbeatslow() {
        blinkIn.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP2_HEARTBEAT_SLOW);
    }

    public void cp2larsonscanner() {
        blinkIn.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP2_LARSON_SCANNER);
    }

    public void cp2lightchase() {
        blinkIn.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP2_LIGHT_CHASE);
    }

    public void cp2() {
        blinkIn.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP2_SHOT);
    }

    public void cp2strobe() {
        blinkIn.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP2_STROBE);
    }





    //fire

    public void fire (){
        blinkIn.setPattern(RevBlinkinLedDriver.BlinkinPattern.FIRE_LARGE);
    }

    public void firelarge() {
        blinkIn.setPattern(RevBlinkinLedDriver.BlinkinPattern.FIRE_LARGE);
    }

    public void firemedium() {
        blinkIn.setPattern(RevBlinkinLedDriver.BlinkinPattern.FIRE_MEDIUM);
    }





    //heartbeat

    public void heartbeatblue() {
        blinkIn.setPattern(RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_BLUE);
    }

    public void heartbeatgray() {
        blinkIn.setPattern(RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_GRAY);
    }

    public void heartbeatred() {
        blinkIn.setPattern(RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_RED);
    }

    public void heartbeatwhite() {
        blinkIn.setPattern(RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_WHITE);
    }





    //larson scanner

    public void larsonscannergray() {
        blinkIn.setPattern(RevBlinkinLedDriver.BlinkinPattern.LARSON_SCANNER_GRAY);
    }

    public void larsonscannerred() {
        blinkIn.setPattern(RevBlinkinLedDriver.BlinkinPattern.LARSON_SCANNER_RED);
    }





    //light chase

    public void lightchaseblue() {
        blinkIn.setPattern(RevBlinkinLedDriver.BlinkinPattern.LIGHT_CHASE_BLUE);
    }

    public void lightchasegray() {
        blinkIn.setPattern(RevBlinkinLedDriver.BlinkinPattern.LIGHT_CHASE_GRAY);
    }

    public void lightchasered() {
        blinkIn.setPattern(RevBlinkinLedDriver.BlinkinPattern.LIGHT_CHASE_RED);
    }





    //rainbow

    public void rainbow(){
        blinkIn.setPattern(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_RAINBOW_PALETTE);
    }

    public void rainbowforest() {
        blinkIn.setPattern(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_FOREST_PALETTE);
    }

    public void rainbowrainbow() {
        blinkIn.setPattern(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_RAINBOW_PALETTE);
    }

    public void rainbowlava() {
        blinkIn.setPattern(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_LAVA_PALETTE);
    }

    public void rainbowocean() {
        blinkIn.setPattern(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_OCEAN_PALETTE);
    }

    public void rainbowparty() {
        blinkIn.setPattern(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_PARTY_PALETTE);
    }

    public void rainbowwithglitter() {
        blinkIn.setPattern(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_WITH_GLITTER);
    }





    //shot

    public void shotblue() {
        blinkIn.setPattern(RevBlinkinLedDriver.BlinkinPattern.SHOT_BLUE);
    }

    public void shotred() {
        blinkIn.setPattern(RevBlinkinLedDriver.BlinkinPattern.SHOT_RED);
    }

    public void shotwhite() {
        blinkIn.setPattern(RevBlinkinLedDriver.BlinkinPattern.SHOT_WHITE);
    }





    //sinelon

    public void sinelonforest() {
        blinkIn.setPattern(RevBlinkinLedDriver.BlinkinPattern.SINELON_FOREST_PALETTE);
    }

    public void sinelonlava() {
        blinkIn.setPattern(RevBlinkinLedDriver.BlinkinPattern.SINELON_LAVA_PALETTE);
    }

    public void sinelonocean() {
        blinkIn.setPattern(RevBlinkinLedDriver.BlinkinPattern.SINELON_OCEAN_PALETTE);
    }

    public void sinelonparty() {
        blinkIn.setPattern(RevBlinkinLedDriver.BlinkinPattern.SINELON_PARTY_PALETTE);
    }

    public void sinelonrainbow() {
        blinkIn.setPattern(RevBlinkinLedDriver.BlinkinPattern.SINELON_RAINBOW_PALETTE);
    }





    //strobe

    public void strobeblue() {
        blinkIn.setPattern(RevBlinkinLedDriver.BlinkinPattern.STROBE_BLUE);
    }

    public void strobegold() {
        blinkIn.setPattern(RevBlinkinLedDriver.BlinkinPattern.STROBE_GOLD);
    }

    public void strobered() {
        blinkIn.setPattern(RevBlinkinLedDriver.BlinkinPattern.STROBE_RED);
    }

    public void strobewhite() {
        blinkIn.setPattern(RevBlinkinLedDriver.BlinkinPattern.STROBE_WHITE);
    }





    //twinkles

    public void twinklesforest() {
        blinkIn.setPattern(RevBlinkinLedDriver.BlinkinPattern.TWINKLES_FOREST_PALETTE);
    }

    public void twinkleslava() {
        blinkIn.setPattern(RevBlinkinLedDriver.BlinkinPattern.TWINKLES_LAVA_PALETTE);
    }

    public void twinklesocean() {
        blinkIn.setPattern(RevBlinkinLedDriver.BlinkinPattern.TWINKLES_OCEAN_PALETTE);
    }

    public void twinklesparty() {
        blinkIn.setPattern(RevBlinkinLedDriver.BlinkinPattern.TWINKLES_PARTY_PALETTE);
    }

    public void twinklesrainbow() {
        blinkIn.setPattern(RevBlinkinLedDriver.BlinkinPattern.TWINKLES_RAINBOW_PALETTE);
    }
}