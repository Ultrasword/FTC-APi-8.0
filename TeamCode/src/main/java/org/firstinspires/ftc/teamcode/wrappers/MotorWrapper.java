package org.firstinspires.ftc.teamcode.wrappers;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.checkerframework.checker.index.qual.LTEqLengthOf;
import org.firstinspires.ftc.teamcode.system.OpModeSGlobals;

import java.util.ArrayList;
import java.util.List;

public class MotorWrapper {

    public static final int POWERMODE = 0, ENCODERMODE = 1;

    // -------------------------------------------------- //
    // constants

    public static final int TICKS_TORQNADO = 1440, TICKS_COREHEX = 240, TICKS_HD_HEX_MOTOR_20_1 = 530;
    public static final double PI = 3.14159265;

    // -------------------------------------------------- //
    // variables
    public MotorRatio motorRatio;
    private DcMotor motor;
    private int deltaTicks, currentTicks;
    private int ticksPerSpin = 1440;

    private double wheelDiameter = 0.0;
    private boolean hasTarget = false;
    private int targetPos = 0, startPos = 0;
    private double mPower = 0.0, mTargetPower = 0.0;

    private int currentRunMode = POWERMODE;

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
        // update ticks -- is good no bad
        int pastTicks = currentTicks;
        currentTicks = getCurrentTicks();
        deltaTicks = currentTicks - pastTicks;
        // check which runmode and then run motor code -- is bad -- big sad
        if (ENCODERMODE == currentRunMode) {
            // encoder runmode
            updateTarget();
        }
        motor.setPower(mPower);
    }

    public boolean failSafeActive(){
        // check if deltaTicks = 0  but  power is not 0
        return deltaTicks == 0 && Math.abs(Math.round(getCurrentMotorPower() * 100.0) / 100.0) > 0;
    }

    public void updateTarget(){
        if(!hasTarget) return;
        // move motor to position
        if(targetPos > startPos){
            // move forward
            setPower(mTargetPower);
        }else if(targetPos < startPos) {
            // move backward
            setPower(-mTargetPower);
        }
    }

    public void setRunMode(int mode){
        if(mode != 0 && mode != 1) setRunMode(0);
        setRunMode(mode);
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
        return wheelDiameter * PI * ((double) motorRatio.findFinalRotation(currentTicks) / (double) (ticksPerSpin));
    }

    public void setPower(double power){
        this.mPower = power;
    }

    public double getPower(){
        return this.mPower;
    }

    public double getCurrentMotorPower(){
        return this.motor.getPower();
    }

    public void setTargetPower(double power){
        this.mTargetPower = power;
    }

    public double getTargetPower(){
        return mTargetPower;
    }

    public void setTargetPosition(int position){
        // stupido moment
        if(hasTarget) return;
        hasTarget = true;
        targetPos = (int)motorRatio.reverseTicksToFinal(position);
        startPos = getCurrentTicks();
        // set ftc api target
        motor.setTargetPosition(targetPos);
    }

    public void setTargetRelative(int relative){
        setTargetPosition(getCurrentTicks() + relative);
//        OpModeSGlobals.telemetry.addData("Testing", "No error at 118 in setTargetRelative()");
//        setTargetPosition(targetPos);
    }

    public int getTargetPosition(){
        return this.targetPos;
    }

    public MotorRatio getMotorRatio(){
        return motorRatio;
    }

    public void setDirection(DcMotorSimple.Direction direction){
        motor.setDirection(direction);
    }
}
