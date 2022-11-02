package org.firstinspires.ftc.teamcode.wrappers;

import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.ArrayList;
import java.util.List;

public class MotorWrapper {
    // -------------------------------------------------- //
    // motor wrapper classes

    public static class Gear{
        public int teeth;
        public Gear(int teeth){
            this.teeth = teeth;
        }

        public double convertRatio(Gear next, double spinTicks){
            return (double)teeth / (double)next.teeth * (double)spinTicks;
        }
    }

    public static class MotorRatio {
        List<Gear> gearList = new ArrayList<Gear>();

        public MotorRatio(Gear[] gears) {
            // add gears to list
            for(Gear gear : gears) addGear(gear);
        }

        public void addGear(Gear gear){
            gearList.add(gear);
        }

        public double findFinalRotation(double spinTicks){
            for (int i = 0; i < gearList.size() - 1; i++)
                spinTicks = gearList.get(i).convertRatio(gearList.get(i+1), spinTicks);
            return spinTicks;
        }

        public double reverseTicksToFinal(double spinTicks){
            for(int i = gearList.size(); i > 1; i--)
                spinTicks = gearList.get(i).convertRatio(gearList.get(i), spinTicks);
            return spinTicks;
        }
    }

    // -------------------------------------------------- //
    // constants

    public static final int TICKS_TORQNADO = 1440, TICKS_COREHEX = 1440;
    public static final double PI = 3.14159265;

    // -------------------------------------------------- //
    // variables
    private DcMotor motor;
    private MotorRatio motorRatio;
    private int deltaTicks, currentTicks;
    private int ticksPerSpin = 1440;

    private double wheelDiameter = 0.0;
    private boolean hasTarget = false, reachedTarget = true;
    private int targetPos = 0, startPos = 0;
    private double mPower = 0.0, mTargetPower = 0.0;

    // -------------------------------------------------- //
    // code

    public MotorWrapper(DcMotor motor, double wheel_diameter, int ticks_per_spin, MotorRatio mRatio){
        this.motor = motor;
        this.motorRatio = mRatio;
        this.wheelDiameter = wheel_diameter;
        this.ticksPerSpin = (ticks_per_spin == 0) ? 1440 : ticks_per_spin;
        // set motor to use encoders
        this.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void update()
    {
        int pastTicks = currentTicks;
        currentTicks = getCurrentTicks();
        deltaTicks = currentTicks - pastTicks;
        // check if there is a target
        if(hasTarget && !reachedTarget)
            updateTarget();
        motor.setPower(mPower);
    }

    public void updateTarget(){
        // move
        if(targetPos > startPos){
            if(currentTicks > targetPos){
                setPower(0);
                reachedTarget = true;
            }
            else{
                setPower(mTargetPower);
            }
        }else if(targetPos < startPos){
            if( currentTicks < targetPos){
                setPower(0);
                reachedTarget = true;
            }else {
                setPower(-mTargetPower);
            }
        }
    }

    public DcMotor getMotor(){
        return motor;
    }

    public int getCurrentTicks(){
        return this.motor.getCurrentPosition();
    }

    public int getDeltaTicks(){
        return deltaTicks;
    }

    public double getDistanceTravelledThisUpdate(){
        return wheelDiameter * PI * ((double)motorRatio.findFinalRotation(deltaTicks) / (double)(ticksPerSpin));
    }

    public double getTotalDistanceTravelled(){
        return wheelDiameter * PI * ((double)motorRatio.findFinalRotation(currentTicks) / (double)(ticksPerSpin));
    }

    public void setPower(double power){
        this.mPower = power;
    }

    public double getPower(){
        return this.mPower;
    }

    public void setTargetPower(double power){
        this.mTargetPower = power;
    }

    public double getTargetPower(){
        return mTargetPower;
    }

    public void setTargetPosition(int position){
        setTargetRelative(currentTicks-position);
    }

    public void setTargetRelative(int relative){
        hasTarget = true;
        reachedTarget = false;
        targetPos += motorRatio.reverseTicksToFinal(relative);
        setTargetPosition(targetPos);
    }

    public int getTargetPosition(){
        return this.targetPos;
    }
}
