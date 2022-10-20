package org.firstinspires.ftc.teamcode.wrappers;

import com.qualcomm.robotcore.hardware.DcMotor;

public class MotorWrapper {
    final double PI = 3.14159265;
    final int MAX_TICKS = 500;

    private DcMotor motor;
    private int passed_ticks = 0, current = 0, past = 0;
    private int TICKS_PER_SPIN = 1440;

    private double DISTANCE_INCHES = 0.0, WHEEL_DIAMETER = 0.0;

    private boolean hasTarget = false, reachedTarget = true;
    private int target = 0, start = 0;
    private double power = 0.0, targetpower = 0.0;

    public MotorWrapper(DcMotor motor, double wheel_diameter, int ticks_per_spin){
        this.motor = motor;
        this.WHEEL_DIAMETER = wheel_diameter;
        this.TICKS_PER_SPIN = (ticks_per_spin == 0) ? 1440 : ticks_per_spin;
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
        if(hasTarget && !reachedTarget){
            // move
            if(target > start){
                if(current > target) {
                    setPower(0);
                    reachedTarget = true;
                }
                else setPower(targetpower);
            }
            else if(target < start){
                if(current < target) {
                    setPower(0);
                    reachedTarget = true;
                }
                else setPower(-targetpower);
            }
        }
        motor.setPower(power);
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

    public double getDistanceTravelledThisUpdate(){
        return WHEEL_DIAMETER * PI * ((double)passed_ticks / (double)(TICKS_PER_SPIN));
    }

    public double getTotalDistanceTravelled(){
        return WHEEL_DIAMETER * PI * ((double)current / (double)(TICKS_PER_SPIN));
    }

    public void setPower(double power){
        this.power = power;
    }

    public double getPower(){
        return this.power;
    }

    public void setTargetPower(double power){
        this.targetpower = power;
    }

    public double getTargetPower(){
        return targetpower;
    }

    public void setTargetPosition(int position){
        hasTarget = true;
        reachedTarget = false;
        this.target = position; this.hasTarget = true; start = current;
    }

    public void setTargetRelative(int relative){
        hasTarget = true;
        reachedTarget = false;
        target += relative;
        setTargetPosition(target);
    }

    public int getTargetPosition(){
        return this.target;
    }
}
