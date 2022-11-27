package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.wrappers.Controller;
import org.firstinspires.ftc.teamcode.wrappers.MecanumChassis;
import org.firstinspires.ftc.teamcode.wrappers.Position;

@Autonomous(name="autoleft")
public class AutoTestLeft extends LinearOpMode {
    MecanumChassis robot;
    Position pos;
    Controller control;
    @Override
    public void runOpMode() throws InterruptedException {
        robot = new MecanumChassis(hardwareMap);
        pos = new Position(robot);
        control = new Controller(robot, pos);
        waitForStart();
        // open claw
        robot.intake.setPosition(0.75);
        sleep(500);
        // set arm position - while moving to high junction - move arm up
        setArmPositionTiming(520,0.1,2000);
        goToTiming(0,1.1,0,1.3,60, 0.02, 2, true);
        // go to position again
        goToTiming(0.1,1.1,69,1.3,50, 0.02, 1, true);
        // release claw
        robot.intake.setPosition(0.55);
        sleep(2000);
        // move to position
        goToTiming(0,1.1,0,1.3,60, 0.02, 2, true);
        // move arm down
        setArmPosition(10,0.2);
        // go to timing
        goToTiming(0,0.57,0,1.3,60,0.02,2,true);
        setArmPosition(0, 0.2);


        // pain

        //        goToTiming(0, 1, 0, 1.3, 120, 0.04, 2, true);
//        setArmPosition(300, 0.3);
//        goToTiming(0,1,90, 1.3, 120, 0.04, 2, true);
//        robot.intake.setPosition(0.55);
//        sleep(300);
//        goToTiming(0,1,0,1.3,120,0.04,2,true);
//        setArmPosition(10,0.2);
//        goToTiming(0,0,0,1.3,120,0.04,2,true);
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
    private void setArmPositionTiming(int pos, double speed, int delay) {
        sleep(delay);
        robot.leftArm.setTargetPosition(pos);
        robot.rightArm.setTargetPosition(pos);
        robot.leftArm.setPower(speed);
        robot.rightArm.setPower(speed);
        robot.leftArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
}
