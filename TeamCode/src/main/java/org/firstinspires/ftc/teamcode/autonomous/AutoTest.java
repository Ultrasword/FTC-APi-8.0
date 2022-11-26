package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.wrappers.Controller;
import org.firstinspires.ftc.teamcode.wrappers.MecanumChassis;
import org.firstinspires.ftc.teamcode.wrappers.Position;

@Autonomous(name="auton")
public class AutoTest extends LinearOpMode {
    MecanumChassis robot;
    Position pos;
    Controller control;
    @Override
    public void runOpMode() throws InterruptedException {
        robot = new MecanumChassis(hardwareMap);
        pos = new Position(robot);
        control = new Controller(robot, pos);
        waitForStart();
        setArmPosition(300, 0.3);
        goToTiming(0,1.3,90, 1, 80, 0.04, 2, true);
        goToTiming(0, 0, -45, 1, 70, 0.04, 2, true);

    }
    private void goToTiming(double x, double y, double angle, double speed, double angleSpeed, double distanceDeadzone, double angleDeadzone, boolean velocityControl) {
        control.goTo(x, y, angle, speed, angleSpeed, distanceDeadzone, angleDeadzone, velocityControl);
        while (!control.finished) sleep(10);
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
