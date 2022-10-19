package org.firstinspires.ftc.teamcode.wrappers;

import com.qualcomm.robotcore.hardware.DcMotor;

public class MotorWrapper {
    final double PI = 3.14159265;
    final int MAX_TICKS = 500;
    final int TICKS_PER_SPIN = 1440;

    private DcMotor motor;
    private int passed_ticks = 0, current = 0, past = 0;

    private boolean hasTarget = false;
    private int target = 0, start = 0;
    private double power = 0.0;

    public MotorWrapper(DcMotor motor){
        this.motor = motor;
        // set motor to use encoders
        this.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void update()
    {
        past = current;
        current = getCurrentTicks();
        passed_ticks = current - past;
        // check if there is a target
        if(hasTarget){
            // move
            if(target > start){
                if(current > target) setPower(0);
                else setPower(power);
            }
            else if(target < start){
                if(current < target) setPower(0);
                else setPower(-power);
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
        return this.passed_ticks;
    }

    public double getDistanceTravelledThisUpdate(int ticksPerRotation, double wheelRadius){
        return wheelRadius * ((double)passed_ticks / (double)(ticksPerRotation));
    }

    public double getTotalDistanceTravelled(int ticksPerRotation, double wheelRadius){
        return wheelRadius * ((double)current / (double)ticksPerRotation);
    }

    public void setPower(double power){
        this.power = power;
    }

    public double getPower(){
        return this.power;
    }

    public void setTargetPosition(int position){
        hasTarget = true;
        this.target = position; this.hasTarget = true; start = current;
    }

    public void setTargetRelative(int relative){
        hasTarget = true;
        target += relative;
        setTargetPosition(target);
    }

    public int getTargetPosition(){
        return this.target;
    }

}
