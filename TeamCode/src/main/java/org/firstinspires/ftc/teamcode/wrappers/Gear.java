package org.firstinspires.ftc.teamcode.wrappers;

public class Gear{
    public int teeth;
    public Gear(int teeth){
        this.teeth = teeth;
    }

    public double convertRatio(Gear next, double spinTicks){
        return (double)teeth / (double)next.teeth * (double)spinTicks;
    }
}
