package org.firstinspires.ftc.teamcode.system;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.wrappers.MotorRatio;
import org.firstinspires.ftc.teamcode.wrappers.MotorWrapper;

public class Lift2Motor extends RobotSystem {


    private MotorWrapper left, right;
    private int maxTicksRotate;
    private int leewayTicks;

    // for arm position stuff
    private boolean toggle = false;
    private int cnt = 0;

    private int restPosition[] = {0,0}, maxPosition[] = {0,0};

    public Lift2Motor(MotorWrapper left, MotorWrapper right, int maxTicksRotate, int leewayTicks, double power){
        // default value for leeWay ticks is 10; (or should be)
        super();
        // set motors
        this.left = left; this.right = right;
        // set motor directions
        this.leewayTicks = leewayTicks;
        this.maxTicksRotate = maxTicksRotate;
        // motor mode
        this.left.setTargetPower(power);
        this.right.setTargetPower(power);
        // lerp
        this.left.setDirection(DcMotorSimple.Direction.FORWARD);
        this.right.setDirection(DcMotorSimple.Direction.REVERSE);
        // set direction
        this.left.setRunMode(MotorWrapper.ENCODERMODE);
        this.right.setRunMode(MotorWrapper.ENCODERMODE);
        // locking
//        this.left.setLockMotor(true);
//        this.right.setLockMotor(true);
        // notes --
        /*
            make sure you set lerping to true
            and also make sure your power is low for the arm motors
         */
        // accounting for deacceleration and cringe...
        this.restPosition[0] = this.leewayTicks;
        this.restPosition[1] = -this.leewayTicks;
        this.maxPosition[0] = this.maxTicksRotate - this.leewayTicks;
        this.maxPosition[1] = -this.maxTicksRotate + this.leewayTicks;
    }

    @Override
    public void update() {
        if(OpModeSGlobals.gamepad1.dpad_up){
            left.setTargetPosition(maxPosition[0]);
            right.setTargetPosition(maxPosition[1]);
        }else if (OpModeSGlobals.gamepad1.dpad_down){
            left.setTargetPosition(restPosition[0]);
            right.setTargetPosition(restPosition[1]);
        }

        // update motorwrappers
        left.update();
        right.update();

    }

    @Override
    public void updateTelemetry(Telemetry telemetry) {
        telemetry.addData("Power", left.getCurrentMotorPower());
        telemetry.addData("Power", left.getPower());
        telemetry.addData("Power", left.getTargetPower());
        telemetry.addData("Current Arm Position: ", String.format("%d, %d", left.getCurrentTicks(), right.getCurrentTicks()));
        telemetry.addData("Target Arm Position: ", String.format("%d, %d", left.getTargetPosition(), right.getTargetPosition()));
    }
}
