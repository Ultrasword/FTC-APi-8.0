package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.system.OpModeSGlobals;
import org.firstinspires.ftc.teamcode.wrappers.MotorRatio;
import org.firstinspires.ftc.teamcode.wrappers.MotorWrapper;

@TeleOp(name = "Two Motor Opmode")
public class TwoMotorWheelsOpMode extends LinearOpMode {

    MotorWrapper left, right;

    @Override
    public void runOpMode() throws InterruptedException {

        OpModeSGlobals.initOpMode(this);

        left = new MotorWrapper(hardwareMap.get(DcMotor.class, "ml"), 5, MotorWrapper.TICKS_COREHEX, new MotorRatio());
        right = new MotorWrapper(hardwareMap.get(DcMotor.class, "mr"), 5, MotorWrapper.TICKS_COREHEX, new MotorRatio());



        telemetry.addData("hi", "gu");
        telemetry.update();

        int amp = 10;
        waitForStart();
        while(opModeIsActive()){
            if(gamepad1.left_stick_y != 0)
                left.setTargetRelative((int)(gamepad1.left_stick_y * amp));
            if(gamepad1.right_stick_y != 0)
                right.setTargetRelative((int)(gamepad1.right_stick_y * amp));

            left.update();
            right.update();

            telemetry.addData("Left Target + Position", String.format("%d, %d", left.getTargetPosition(), left.getCurrentTicks()));
            telemetry.addData("RightTarget + Position", String.format("%d, %d", left.getTargetPosition(), left.getCurrentTicks()));

            telemetry.update();
        }
    }


}
