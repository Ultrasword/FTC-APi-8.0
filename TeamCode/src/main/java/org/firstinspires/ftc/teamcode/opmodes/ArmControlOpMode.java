package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.wrappers.Gear;
import org.firstinspires.ftc.teamcode.wrappers.MotorRatio;
import org.firstinspires.ftc.teamcode.wrappers.MotorWrapper;

@TeleOp(name="ArmTestOpMode")


public class ArmControlOpMode extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        // setup
        // arms are 1:1 ratio so no need for motor ratios
        MotorWrapper left, right;
        left = new MotorWrapper(hardwareMap.get(DcMotor.class, "fl"), 2.0, 0, new MotorRatio());
        right = new MotorWrapper(hardwareMap.get(DcMotor.class, "fr"), 2.0, 0, new MotorRatio());

        // prerun
        waitForStart();
        left.setTargetPower(0.1);
        right.setTargetPower(0.1);
        left.setLerping(true);
        right.setLerping(true);
        left.setDirection(DcMotorSimple.Direction.FORWARD);
        right.setDirection(DcMotorSimple.Direction.REVERSE);

        left.setRunMode(MotorWrapper.ENCODERMODE);
        right.setRunMode(MotorWrapper.ENCODERMODE);

        int MAXARMPOS = MotorWrapper.TICKS_TORQNADO / 3;

        boolean toggle = false; int cnt = 0;

        // run loop
        while (opModeIsActive()){
            if(gamepad1.x){
                if(toggle) continue;
                cnt++;
                toggle = true;
                if(cnt % 2 != 0) {
                    left.setTargetPosition(80);
                    right.setTargetPosition(80);
                }else {
                    left.setTargetPosition(10);
                    right.setTargetPosition(10);
                }
            }else{
                toggle = false;
            }
            left.update();
            right.update();

            telemetry.addData("Distance Travelled", String.format("%.2f, %.2f", left.getTotalDistanceTravelled(), right.getTotalDistanceTravelled()));
            telemetry.addData("TargetPower", String.format("%.2f, %.2f", left.getTargetPower(), right.getTargetPower()));
            telemetry.addData("Power", String.format("%.2f, %.2f", left.getCurrentMotorPower(), right.getCurrentMotorPower()));
            telemetry.addData("Target_Pos", String.format("%d, %d", left.getTargetPosition(), right.getTargetPosition()));
            telemetry.addData("Arm_Pos:", String.format("%d, %d", left.getCurrentTicks(), right.getCurrentTicks()));
            telemetry.update();
            sleep(50);
        }
    }
}
