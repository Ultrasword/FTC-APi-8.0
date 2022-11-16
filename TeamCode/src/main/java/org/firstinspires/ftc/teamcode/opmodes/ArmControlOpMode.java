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
        arm = new MotorWrapper(hardwareMap.get(DcMotor.class, "fl"), 2.0, 0, new MotorRatio());
//        arm.motorRatio.addGear(new Gear(100));
        // prerun
        waitForStart();
        arm.setTargetPower(0.1);

        int MAXARMPOS = MotorWrapper.TICKS_COREHEX / 3;

        boolean toggle = false; int cnt = 0;

        // run loop
        while (opModeIsActive()){
            arm.update();
            if(gamepad1.x){
                if(toggle) continue;
                cnt++;
                toggle = true;
                if(cnt % 2 != 0) {
                    arm.setTargetPosition(80);
                }else {
                    arm.setTargetPosition(10);
                }
            }else{
                toggle = false;
            }

            telemetry.addData("Count", cnt);
            telemetry.addData("Toggle", toggle);
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
