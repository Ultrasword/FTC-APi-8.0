package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.wrappers.MotorWrapper;

@TeleOp(name="ArmTestOpMode")


public class ArmControlOpMode extends LinearOpMode {

    MotorWrapper arm;

    @Override
    public void runOpMode() throws InterruptedException {
        // setup
        arm = new MotorWrapper(hardwareMap.get(DcMotor.class, "m1"));
        // prerun
        waitForStart();

        // run loop
        while (opModeIsActive()){
            arm.update();
            if (gamepad1.dpad_up){
                arm.setTargetRelative(10);
            }else if (gamepad1.dpad_down){
                arm.setTargetRelative(-10);
            }
        }
    }
}
