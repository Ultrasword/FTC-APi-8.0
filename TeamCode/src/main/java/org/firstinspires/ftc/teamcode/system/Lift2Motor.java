package org.firstinspires.ftc.teamcode.system;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.wrappers.MotorRatio;
import org.firstinspires.ftc.teamcode.wrappers.MotorWrapper;

public class Lift2Motor extends System {

    private MotorWrapper left, right;
    private int maxTicksRotate;

    public Lift2Motor(String m1, String m2, int maxTicksRotate){
        super();
        left = new MotorWrapper(OpModeSGlobals.hwMap.get(DcMotor.class, m1), 0.0, MotorWrapper.TICKS_TORQNADO, new MotorRatio());
        right = new MotorWrapper(OpModeSGlobals.hwMap.get(DcMotor.class, m2), 0.0, MotorWrapper.TICKS_TORQNADO, new MotorRatio());
        // set max rotation for arms
        this.maxTicksRotate = maxTicksRotate;
    }

    @Override
    public void update() {
        // check if we need to spin
        if(OpModeSGlobals.gamepad1.y){
            // set rotating stuff

        }
        // update motorwrappers
        left.update(); right.update();

    }

}
