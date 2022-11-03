package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.wrappers.Gear;
import org.firstinspires.ftc.teamcode.wrappers.MotorRatio;
import org.firstinspires.ftc.teamcode.wrappers.MotorWrapper;

@TeleOp(name="ArmTestOpMode")


public class ArmControlOpMode extends LinearOpMode {

    MotorWrapper arm;

    @Override
    public void runOpMode() throws InterruptedException {
        // setup
        arm = new MotorWrapper(hardwareMap.get(DcMotor.class, "m1"), 2.0, 0,
                new MotorRatio(new Gear[]{new Gear(MotorWrapper.TICKS_TORQNADO)}));
        // prerun
        waitForStart();
        arm.setTargetPower(0.4);

        // run loop
        while (opModeIsActive()){
            arm.update();
            if (gamepad1.dpad_up){
                telemetry.addData("ArmUp", "");
                arm.setTargetRelative(4);
            }else if (gamepad1.dpad_down){
                telemetry.addData("ArmDown", "");
                arm.setTargetRelative(-4);
            }

            telemetry.addData("Distance Travelled", arm.getTotalDistanceTravelled());
            telemetry.addData("TargetPower", arm.getTargetPower());
            telemetry.addData("Power", arm.getPower());
            telemetry.addData("Target_Pos", arm.getTargetPosition());
            telemetry.addData("Arm_Pos:", arm.getCurrentTicks());
            telemetry.update();
            sleep(50);
        }
    }
}
