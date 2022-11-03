package org.firstinspires.ftc.teamcode.wrappers;

import java.util.ArrayList;
import java.util.List;

public class MotorRatio {
    private List<Gear> gearList = new ArrayList<Gear>();

    public MotorRatio(Gear[] gears) {
        // add gears to list
        for(Gear gear : gears) addGear(gear);
    }

    public void addGear(Gear gear){
        gearList.add(gear);
    }

    public double findFinalRotation(double spinTicks){
        if(gearList.size() <= 1) return spinTicks;
        for (int i = 0; i < gearList.size() - 1; i++)
            spinTicks = gearList.get(i).convertRatio(gearList.get(i+1), spinTicks);
        return spinTicks;
    }

    public double reverseTicksToFinal(double spinTicks){
        if(gearList.size() <= 1) return spinTicks;
        for(int i = gearList.size(); i > 1; i--)
            spinTicks = gearList.get(i).convertRatio(gearList.get(i), spinTicks);
        return spinTicks;
    }
}