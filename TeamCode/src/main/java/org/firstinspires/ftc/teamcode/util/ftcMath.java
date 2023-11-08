package org.firstinspires.ftc.teamcode.util;

public class ftcMath {
    public int ticksperinch(int tickperrotation,double wheeldiameterinmm){
        return (int)(tickperrotation*(0.0393701*(Math.PI*2*wheeldiameterinmm)));
    }
    public int ticksforlenght(int ticksperrev,int wheeldiaminmm,double distanceneeded){
        return (int)(distanceneeded*ticksperinch(ticksperrev,wheeldiaminmm));
    }
}

