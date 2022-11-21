package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.wrappers.Gear;
import org.firstinspires.ftc.teamcode.wrappers.MotorRatio;
import org.firstinspires.ftc.teamcode.wrappers.MotorWrapper;

@TeleOp(name="1ArmTestMode")
public class Arm1LiftOpMode extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        // setup
        // arms are 1:1 ratio so no need for motor ratios
        MotorWrapper left, right;
        left = new MotorWrapper(hardwareMap.get(DcMotor.class, "fl"), 2.0, 0, new MotorRatio());

        // prerun
        waitForStart();
        left.setTargetPower(0.2);
        left.setLerping(true);
        left.setDirection(DcMotorSimple.Direction.FORWARD);
//        left.setMotorLocking(false);

        left.setRunMode(MotorWrapper.ENCODERMODE);

        int MAXARMPOS = MotorWrapper.TICKS_COREHEX / 3;
        MAXARMPOS = 50;
        boolean toggle = false; int cnt = 0;
        // run loop
        while (opModeIsActive()){
            if(gamepad1.x){
                if(!toggle) {
                    toggle = true;
                    cnt++;
                    if (cnt % 2 != 0) {
                        left.setTargetPosition(MAXARMPOS);
                    } else {
                        left.setTargetPosition(0);
                    }
                }
            }else{
                toggle = false;
            }
            left.update();

            telemetry.addData("Distance Travelled", String.format("%.2f", left.getTotalDistanceTravelled()));
            telemetry.addData("TargetPower", String.format("%.2f", left.getTargetPower()));
            telemetry.addData("Power", String.format("%.2f", left.getCurrentMotorPower()));
            telemetry.addData("Target_Pos", String.format("%d", left.getTargetPosition()));
            telemetry.addData("Arm_Pos:", String.format("%d", left.getCurrentTicks()));
            telemetry.update();
            sleep(50);
        }
    }
}
