package org.firstinspires.ftc.teamcode.system;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.wrappers.IMUWrapper;
import org.firstinspires.ftc.teamcode.wrappers.MotorRatio;
import org.firstinspires.ftc.teamcode.wrappers.MotorWrapper;

public class DriveTrain extends RobotSystem {
    /* gear ratio:

    motor -> gearbox = 20:1
    gearbox -> wheel = 1:1

    */

    private MotorWrapper fl, fr, bl, br;
    private double x, y, rx, dn;

    private IMUWrapper imuWrapper;


    public DriveTrain(MotorWrapper fl, MotorWrapper fr, MotorWrapper bl, MotorWrapper br) {
        this.fl = fl; this.fr = fr; this.bl = bl; this.br = br;

        // reverse right side motors for positive movement on both sides
        this.fr.setDirection(DcMotorSimple.Direction.REVERSE);
        this.br.setDirection(DcMotorSimple.Direction.REVERSE);

//        imuWrapper = new IMUWrapper("imu");
        // setup imu stuffs?
    }

    @Override
    public void update() {
        // slow or fast mode stuff
        if(OpModeSGlobals.gamepad1.right_trigger>0.1)
            OpModeSGlobals.slowMode = true;
        else OpModeSGlobals.slowMode = false;

        // mecanum drive vroom
        // two stick drive - benefits of only controlling one thing at a time
        y = -OpModeSGlobals.gamepad1.left_stick_y;
        x = OpModeSGlobals.gamepad1.left_stick_x;
        rx = OpModeSGlobals.gamepad1.right_stick_x;

        // dn to ensure constant power ratios
        dn = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

        // get imu orientations
//        double yaw = imuWrapper.getYaw();
        // if yaw < 0; we want spin right
        // if yaw > 0; we want spin left
        // this depends on if we need to orient
        // create a function that determines power offset when moving

        double flp, frp, blp, brp;
        flp = (y + x - rx) / dn;
        frp = (y - x + rx) / dn;
        blp = (y - x - rx) / dn;
        brp = (y + x + rx) / dn;

        // the coef :)
        if (OpModeSGlobals.slowMode){
            flp *= OpModeSGlobals.slowModeCoef;
            frp *= OpModeSGlobals.slowModeCoef;
            blp *= OpModeSGlobals.slowModeCoef;
            brp *= OpModeSGlobals.slowModeCoef;
        }

        fl.setPower(flp);
        fr.setPower(frp);
        bl.setPower(blp);
        br.setPower(brp);

        fl.update();
        fr.update();
        bl.update();
        br.update();

    }

    @Override
    public void updateTelemetry(Telemetry telemetry){
        telemetry.addData("Motor Power", String.format("fl: %.2f | fr: %.2f | bl: %.2f | br: %.2f", fl.getCurrentMotorPower(),
                                    fr.getCurrentMotorPower(), bl.getCurrentMotorPower(), br.getCurrentMotorPower()));
    }


}
