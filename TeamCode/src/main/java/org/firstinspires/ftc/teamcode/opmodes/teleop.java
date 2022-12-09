package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.wrappers.Controller;
import org.firstinspires.ftc.teamcode.wrappers.MecanumChassis;
import org.firstinspires.ftc.teamcode.wrappers.Position;

@TeleOp(name="teleop")
public class teleop extends LinearOpMode {
    private MecanumChassis robot;
    private Position pos;
    private Controller control;
    private double clamp(double min, double max, double val) {
        if (val<min) return min;
        else if (val>max) return max;
        else return val;

    }
    @Override
    public void runOpMode() throws InterruptedException {
        int currentArmPosition=0;
        robot = new MecanumChassis(hardwareMap);
        pos = new Position(robot);
        control = new Controller(robot, pos);
        waitForStart();
        setArmPosition(20, 0.3);
        while (opModeIsActive()) {
            telemetry.addData("Servo Position ", robot.intake.getPosition());
            telemetry.addData("Arm Position ", robot.leftArm.getCurrentPosition());
            telemetry.addData("Position Data", String.format("%.2f %.2f %.2f",pos.x,pos.y,pos.angle));
            telemetry.update();
            double lx = gamepad1.left_stick_x, ly = gamepad1.left_stick_y, rx = gamepad1.right_stick_x, ry = gamepad2.right_stick_y;
            double dn = 0.8/Math.max(Math.abs(lx)+0.7*Math.abs(rx)+Math.abs(ly),1);
            control.goTo(pos.x+0.3*lx,pos.y+0.3*ly,pos.angle+rx*5,1.5*Math.sqrt(lx*lx+ly*ly),ry*180,0.02,2,false);
//            robot.fr.setPower((ly+lx+0.7*rx)*dn);
//            robot.fl.setPower((ly-lx-0.7*rx)*dn);
//            robot.br.setPower((ly-lx+0.7*rx)*dn);
//            robot.bl.setPower((ly+lx-0.7*rx)*dn);
            if (gamepad2.dpad_up) {
                currentArmPosition = 520;
                setArmPosition(currentArmPosition, 0.3);
            } else if (gamepad2.b) {
                currentArmPosition = 365;
                setArmPosition(currentArmPosition, 0.3);
            } else if (gamepad2.y) {
                currentArmPosition = 260;
                setArmPosition(currentArmPosition, 0.3);
            } else if (gamepad2.x) {
                currentArmPosition = 70;
                setArmPosition(currentArmPosition, 0.3);
            } else if (gamepad2.dpad_down) {
                currentArmPosition = 20;
                setArmPosition(currentArmPosition, 0.2);
            } else if (Math.abs(ry)>0.1) {
                currentArmPosition=(int)clamp(10,520,robot.leftArm.getCurrentPosition()-20*ry);
                setArmPosition(currentArmPosition,Math.abs(ry)*(ry>0?0.1:0.2));
            }
            if (gamepad2.left_bumper) robot.intake.setPosition(0.55);
            else robot.intake.setPosition(0.75);
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
