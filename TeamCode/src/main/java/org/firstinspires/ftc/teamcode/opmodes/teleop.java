package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="darren")
public class teleop extends LinearOpMode {
    MecanumChassis robot;
    @Override
    public void runOpMode() throws InterruptedException {
        robot = new MecanumChassis(hardwareMap);
        waitForStart();
        while (opModeIsActive()) {
            double lx = gamepad1.left_stick_x, ly = gamepad1.left_stick_y, rx = gamepad1.right_stick_x;
            double dn = 1/Math.max(Math.abs(lx)+Math.abs(rx)+Math.abs(ly),1);
//            robot.fr.setPower((ly-lx-rx)*dn);
//            robot.fl.setPower((ly+lx+rx)*dn);
//            robot.br.setPower((ly+lx-rx)*dn);
//            robot.bl.setPower((ly-lx+rx)*dn);

            if (gamepad1.dpad_up) {
                setArmPosition(400, 0.6);
            } else if (gamepad1.dpad_down) {
                setArmPosition(0, 0.6);
            }
            if (gamepad1.left_trigger>0.2) {
                robot.intake.setPower(gamepad1.left_trigger);
            } else robot.intake.setPower(0);
        }
    }
    private void setArmPosition(int pos, double speed) {
        robot.left_arm.setTargetPosition(pos);
        robot.right_arm.setTargetPosition(pos);
        robot.left_arm.setPower(speed);
        robot.right_arm.setPower(speed);
        robot.left_arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.right_arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
}
