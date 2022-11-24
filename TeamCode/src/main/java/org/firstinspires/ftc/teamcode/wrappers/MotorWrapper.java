package org.firstinspires.ftc.teamcode.wrappers;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.checkerframework.checker.index.qual.LTEqLengthOf;
import org.firstinspires.ftc.teamcode.system.OpModeSGlobals;
import org.firstinspires.ftc.teamcode.system.RobotMath;

import java.util.ArrayList;
import java.util.List;

public class MotorWrapper {

    public static final int POWERMODE = 0, ENCODERMODE = 1;

    // -------------------------------------------------- //
    // constants

    public static final int TICKS_TORQNADO = 1440, TICKS_COREHEX = 240, TICKS_HD_HEX_MOTOR_20_1 = 530;
    public static final double PI = 3.14159265;

    public static final double DEF_COEF = 0.5;

    public static final double TETRIX_WHEEL_DIAMETER = 3.9;

    // -------------------------------------------------- //
    // variables
    public MotorRatio motorRatio;
    private DcMotor motor;
    private int deltaTicks, currentTicks;
    private int ticksPerSpin = 1440;

    private double wheelDiameter = 0.0;
    private boolean hasTarget = false, reachedTarget = false;
    private int targetPos = 0, startPos = 0;
    private double cPower = 0.0, mPower = 0.0, mTargetPower = 0.0;
    private int currentRunMode = POWERMODE;

    private double deaccelerationCoef = DEF_COEF;
    private boolean lerping = false, lockMotor = false;

    private PID pid;

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
        this.pid = new PID(1.0, 1.0, 1.0);
    }

    public void update() {
        int pastTicks = currentTicks;
        currentTicks = getCurrentTicks();
        deltaTicks = currentTicks - pastTicks;
        // check which runmode and then run motor code -- is bad -- big sad
        if (ENCODERMODE == currentRunMode)
            // encoder runmode
            updateTarget();
        // make sure it doesnt overflow -- cap to 4 decimal places
        if(lerping) cPower = (double)Math.round(RobotMath.lerp(cPower, mPower, deaccelerationCoef) * 10000.0) / 10000.0;
        else cPower = mPower;
        motor.setPower(cPower);
    }

    public boolean failSafeActive(){
        // check if deltaTicks = 0  but  power is not 0
        // rounds up
        return deltaTicks == 0 && Math.abs(Math.round(getCurrentMotorPower() * 100.0) / 100.0) > 0;
    }

    public void updateTarget(){
        if(!hasTarget) return;
        // move motor to position
        if(targetPos > startPos && currentTicks <= targetPos){
            // move forward
//            setPower(pid.calculatePower(currentTicks, targetPos));
            setPower(mTargetPower);
        }else if(targetPos < startPos && currentTicks >= targetPos) {
            // move backward
//            setPower(pid.calculatePower(currentTicks, targetPos));
            setPower(-mTargetPower);
        }else{
            hasTarget = false;
            reachedTarget = true;
        }
    }

    public void setRunMode(int mode){
        if(mode != 0 && mode != 1) currentRunMode = POWERMODE;
        else currentRunMode = mode;
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

    public void setTargetDistance(double inches){
        double circ = wheelDiameter * PI * 2;
        double targetTicks = (inches / circ) * ticksPerSpin;
        setTargetRelative((int)targetTicks);
    }

    public void setPower(double power){
        this.mPower = power;
    }

    public double getCurrentWrapperPower() {return this.cPower;}

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
        hasTarget = true; reachedTarget = false;
        targetPos = (int)motorRatio.reverseTicksToFinal(position);
        startPos = getCurrentTicks();
        // set ftc api target
        motor.setTargetPosition(targetPos);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
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

    public boolean getReachedTarget(){return this.reachedTarget;}

    public void setDeAccelCoef(double coef){
        this.deaccelerationCoef = coef;
    }

    public double getDeAccelCoef(){
        return this.deaccelerationCoef;
    }

    public void setLerping(boolean lerping){
        this.lerping = lerping;
    }

    public boolean isLerping(){
        return lerping;
    }

    public void setLockMotor(boolean locking){
        this.lockMotor = locking;
        if (locking) this.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        else this.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    public boolean isMotorLocked(){
        return this.lockMotor;
    }

}
