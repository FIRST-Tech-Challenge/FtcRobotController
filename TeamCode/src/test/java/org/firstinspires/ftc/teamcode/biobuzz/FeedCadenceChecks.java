package org.firstinspires.ftc.teamcode.biobuzz;
public final class FeedCadenceChecks {
    private static void check(boolean b,String why){if(!b)throw new AssertionError(why);}
    public static void main(String[] args){
        FeedCadence f=new FeedCadence();
        check(f.apply(1,0)==1&&f.apply(1,.17)==1,"Start one calibrated pocket pulse");
        check(f.apply(1,.18)==0&&f.apply(1,.67)==0,"Mandatory half-second OFF gap");
        check(f.apply(1,.681)==1,"Next pulse after full gap");
        check(f.apply(0,.70)==0&&f.apply(1,.9)==0,"Release/tapping cannot bypass gap");
        check(f.apply(1,1.201)==1,"Resume after cooldown");
        check(f.apply(-1,1.22)==-1&&f.apply(1,1.5)==0,"Reverse immediate; forward cooldown preserved");
        check(f.apply(1,1.721)==1,"Forward resumes after reverse gap");
        f=new FeedCadence();f.apply(1,0);
        check(f.apply(1,2)==0&&f.apply(1,2.49)==0&&f.apply(1,2.501)==1,"Late loop still gives full OFF gap");
        check(f.apply(1,Double.NaN)==0&&f.apply(1,3)==0,"Invalid clock fails OFF");
        System.out.println("Feed pulse, half-second OFF gap, held trigger, tapping, reverse, late-loop and clock-safety checks passed.");
    }
}
