package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.wrappers.MecanumChassis;

@TeleOp(name="teleop")
public class teleop extends LinearOpMode {
    MecanumChassis robot;
    @Override
    public void runOpMode() throws InterruptedException {
        robot = new MecanumChassis(hardwareMap);
        waitForStart();
        while (opModeIsActive()) {
            telemetry.addData("Servo Position ", robot.intake.getPosition());
            telemetry.update();
            double lx = gamepad1.left_stick_x, ly = gamepad1.left_stick_y, rx = gamepad1.right_stick_x;
            double dn = 1/Math.max(Math.abs(lx)+Math.abs(rx)+Math.abs(ly),1);
            robot.fr.setPower((ly-lx-rx)*dn);
            robot.fl.setPower((ly+lx+rx)*dn);
            robot.br.setPower((ly+lx-rx)*dn);
            robot.bl.setPower((ly-lx+rx)*dn);

            if (gamepad1.dpad_up) setArmPosition(400, 0.6);
            else if (gamepad1.dpad_down) setArmPosition(0, 0.6);

            if (gamepad1.left_bumper) robot.intake.setPosition(0.7);
            else robot.intake.setPosition(0.3);
        }
    }
    private void setArmPosition(int pos, double speed) {
        robot.leftArm.setTargetPosition(pos);
        robot.rightArm.setTargetPosition(pos);
        robot.leftArm.setPower(speed);
        robot.rightArm.setPower(speed);
        robot.leftArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
}
