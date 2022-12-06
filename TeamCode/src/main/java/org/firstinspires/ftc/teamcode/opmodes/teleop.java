package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.wrappers.MecanumChassis;
import org.firstinspires.ftc.teamcode.wrappers.Position;

@TeleOp(name="teleop")
public class teleop extends LinearOpMode {
    MecanumChassis robot;
    Position pos;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new MecanumChassis(hardwareMap);
        pos = new Position(robot);
        waitForStart();
        setArmPosition(20, 0.3);
        while (opModeIsActive()) {
            telemetry.addData("Servo Position ", robot.intake.getPosition());
            telemetry.addData("Arm Position ", robot.leftArm.getCurrentPosition());
            telemetry.addData("Position Data", String.format("%.2f %.2f %.2f",pos.x,pos.y,pos.angle));
            telemetry.update();
            double lx = gamepad1.left_stick_x, ly = gamepad1.left_stick_y, rx = gamepad1.right_stick_x, ry = gamepad2.right_stick_y, currentArmPosition = 0;
            double dn = 0.8/Math.max(Math.abs(lx)+0.7*Math.abs(rx)+Math.abs(ly),1);
            robot.fr.setPower((ly+lx+0.7*rx)*dn);
            robot.fl.setPower((ly-lx-0.7*rx)*dn);
            robot.br.setPower((ly-lx+0.7*rx)*dn);
            robot.bl.setPower((ly+lx-0.7*rx)*dn);

            if (Math.abs(ry)>0.1) {
                robot.rightArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.leftArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.rightArm.setPower(-ry*0.1);
                robot.leftArm.setPower(-ry*0.1);
            } else {
                robot.rightArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.leftArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.rightArm.setPower(0);
                robot.leftArm.setPower(0);
            }

            if (gamepad2.dpad_up) {
                setArmPosition(520, 0.3);
                currentArmPosition = 520;
            } else if (gamepad2.b) {
                setArmPosition(365, 0.3);
                currentArmPosition = 365;
            } else if (gamepad2.y) {
                setArmPosition(260, 0.3);
                currentArmPosition = 260;
            } else if (gamepad2.x) {
                setArmPosition(70, 0.3);
                currentArmPosition = 70;
            } else if (gamepad2.dpad_down) {
                setArmPosition(20, 0.2);
                currentArmPosition = 20;
            }
            if (gamepad2.left_bumper) robot.intake.setPosition(0.55);
            else robot.intake.setPosition(0.75);
            if (Math.abs(ry)>0.1) setArmPosition((int)clamp(5,10,currentArmPosition+30),Math.abs(ry)*0.4);

        }
    }
    private double clamp(double min, double max, double val) {
        if (val<min) return min;
        else if (val>max) return max;
        else return val;

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
