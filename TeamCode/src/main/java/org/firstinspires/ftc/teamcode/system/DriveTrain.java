package org.firstinspires.ftc.teamcode.system;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.wrappers.MotorRatio;
import org.firstinspires.ftc.teamcode.wrappers.MotorWrapper;

public class DriveTrain extends System{
    /* gear ratio:

    motor -> gearbox = 20:1
    gearbox -> wheel = 1:1

    */

    private MotorWrapper fl, fr, bl, br;
    private double x, y, rx, dn;


    public DriveTrain(String fl, String fr, String bl, String br) {
        this.fl = new MotorWrapper(OpModeSGlobals.opmode.hardwareMap.get(DcMotor.class, fl), OpModeSGlobals.DRIVETRAIN_WHEEL_DIAMETER_INCHES,
                    MotorWrapper.TICKS_HD_HEX_MOTOR_20_1, new MotorRatio());
        this.fr = new MotorWrapper(OpModeSGlobals.opmode.hardwareMap.get(DcMotor.class, fl), OpModeSGlobals.DRIVETRAIN_WHEEL_DIAMETER_INCHES,
                MotorWrapper.TICKS_HD_HEX_MOTOR_20_1, new MotorRatio());
        this.bl = new MotorWrapper(OpModeSGlobals.opmode.hardwareMap.get(DcMotor.class, fl), OpModeSGlobals.DRIVETRAIN_WHEEL_DIAMETER_INCHES,
                MotorWrapper.TICKS_HD_HEX_MOTOR_20_1, new MotorRatio());
        this.br = new MotorWrapper(OpModeSGlobals.opmode.hardwareMap.get(DcMotor.class, fl), OpModeSGlobals.DRIVETRAIN_WHEEL_DIAMETER_INCHES,
                MotorWrapper.TICKS_HD_HEX_MOTOR_20_1, new MotorRatio());

        // reverse right side motors for positive movement on both sides
        this.fr.setDirection(DcMotorSimple.Direction.REVERSE);
        this.br.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void update() {
        // mecanum drive vroom
        // two stick drive - benefits of only controlling one thing at a time
        y = -OpModeSGlobals.gamepad1.left_stick_y;
        x = OpModeSGlobals.gamepad1.left_stick_x;
        rx = OpModeSGlobals.gamepad1.right_stick_x;

        // dn to ensure constant power ratios
        dn = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

        double flp, frp, blp, brp;
        flp = (y + x + rx) / dn;
        frp = (y - x + rx) / dn;
        blp = (y - x - rx) / dn;
        brp = (y + x - rx) / dn;

        fl.setPower(flp);
        fr.setPower(frp);
        bl.setPower(blp);
        br.setPower(brp);

    }

    @Override
    public void updateTelemetry(Telemetry telemetry){
        telemetry.addData("Power", "");
        telemetry.addData("FL", fl.getPower());
        telemetry.addData("FR", fr.getPower());
        telemetry.addData("BL", bl.getPower());
        telemetry.addData("BR", br.getPower());
        telemetry.addData("Values", String.format("x = %.2f | y = %.2f | rx = %.2f", x, y, rx));
    }


}
