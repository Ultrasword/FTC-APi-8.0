package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.wrappers.Clock;
import org.firstinspires.ftc.teamcode.wrappers.LoggingSystem;
import org.firstinspires.ftc.teamcode.wrappers.MecanumChassis;
import org.firstinspires.ftc.teamcode.wrappers.Position;

import java.util.Date;

@TeleOp(name="raymond telemetry saver")
public class teleoptelemetrystore extends LinearOpMode {
    MecanumChassis robot;
    Position pos;

    double coefficient = 0.65;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new MecanumChassis(hardwareMap);
        pos = new Position(robot);
        LoggingSystem logger = new LoggingSystem();
        telemetry.addData("Logger", logger.success);
        telemetry.addData("Filename", logger.dateFormat.format(new Date()));
        telemetry.update();
        waitForStart();
        setArmPosition(20, 0.3);

        // hard code
        Clock clock = new Clock();
        clock.start();
        double angularVel = 0.0, prevAngle = 0.0, currentAngle = 0.0;

        while (opModeIsActive()) {
            clock.update();
            telemetry.addData("Servo Position ", robot.intake.getPosition());
            telemetry.addData("Arm Target", robot.leftArm.getTargetPosition());
            telemetry.addData("Arm Position ", robot.leftArm.getCurrentPosition());
            telemetry.addData("Position Data", String.format("%.2f %.2f %.2f",pos.x,pos.y,pos.angle));
            telemetry.update();

            // log position
            logger.logData(String.format("X: %.3f, Y: %.3f, ", pos.x, pos.y));
            prevAngle = currentAngle;
            currentAngle = pos.angle;
            angularVel = (currentAngle - prevAngle) / clock.deltaTime;
            logger.logData(String.format("AngulVel: %.3f\n", angularVel));

            double lx = gamepad1.left_stick_x, ly = gamepad1.left_stick_y, rx = gamepad1.right_stick_x, ry = gamepad2.right_stick_y;
            double dn = 0.8/Math.max(Math.abs(lx)+0.7*Math.abs(rx)+Math.abs(ly),1);
            robot.fr.setPower((ly+lx+0.7*rx)*dn * coefficient);
            robot.fl.setPower((ly-lx-0.7*rx)*dn * coefficient);
            robot.br.setPower((ly-lx+0.7*rx)*dn * coefficient);
            robot.bl.setPower((ly+lx-0.7*rx)*dn * coefficient);

//            if (Math.abs(ry)>0.1) {
//                robot.rightArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//                robot.leftArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//                robot.rightArm.setPower(-ry*0.1);
//                robot.leftArm.setPower(-ry*0.1);
//            } else {
//                robot.rightArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//                robot.leftArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//                robot.rightArm.setPower(0);
//                robot.leftArm.setPower(0);
//            }

            if (gamepad2.dpad_up) setArmPosition(520, 0.3);
            else if (gamepad2.b) setArmPosition(365, 0.3);
            else if (gamepad2.y) setArmPosition(260, 0.3);
            else if (gamepad2.x) setArmPosition(70, 0.3);
            else if (gamepad2.dpad_down) setArmPosition(5, 0.2);
                // either stick works for this
            else if (gamepad2.right_stick_y > 0 || gamepad2.left_stick_y > 0) setArmPosition(clamp(5, 520, robot.leftArm.getCurrentPosition() - 35), 0.4);
            else if(gamepad2.right_stick_y < 0 || gamepad2.left_stick_y < 0) setArmPosition(clamp(5, 520, robot.leftArm.getCurrentPosition() + 35), 0.4);

            // open
            if (gamepad2.left_bumper) robot.intake.setPosition(0.55);
                // close
            else robot.intake.setPosition(0.78);
        }
        logger.closeLog();


    }
    public double clamp(double min, double max, double val){
        if(val<min) return min; else if(val > max) return max; return val;
    }
    public int clamp(int min, int max, int val){
        if(val<min) return min; else if(val > max) return max; return val;
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
