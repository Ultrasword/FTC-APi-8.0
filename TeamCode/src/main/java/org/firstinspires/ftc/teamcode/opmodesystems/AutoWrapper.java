package org.firstinspires.ftc.teamcode.opmodesystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.wrappers.Controller;
import org.firstinspires.ftc.teamcode.wrappers.DetectPole;
import org.firstinspires.ftc.teamcode.wrappers.MecanumChassis;
import org.firstinspires.ftc.teamcode.wrappers.Position;
import org.firstinspires.ftc.teamcode.wrappers.Vision;
import org.openftc.easyopencv.OpenCvCamera;

public abstract class AutoWrapper extends LinearOpMode {
    protected MecanumChassis robot;
    protected Position pos;
    protected Controller control;
    protected Vision sleeveDetection;
    protected DetectPole poleDetection;
    protected WebcamName webcamName;
    protected OpenCvCamera camera;
    protected String route;

    protected double MPerFoot = 0.3048, MatSize = MPerFoot*2;

    protected int coneHeightDif = 20, coneCount = 10;
    protected double armSpeed = 0.4, movementSpeed = 0.8, angleSpeed = 50;
    protected int armTop = 520, armDrop = 350, armBottom = 10, armMedHeight = 100;
    protected double angleDeadZone = 2, moveDeadZone = 0.04;



    private void resetCycle(double angle){
        goToDefault(0, MatSize*2, angle);
    }
    private void goToDefault(double x, double y, double angle){
        goTo(x, y, angle, movementSpeed, angleSpeed, moveDeadZone, angleDeadZone, true);
    }
    private void closeIntake() {
        robot.intake.setPosition(0.80);
    }
    private void openIntake() {
        robot.intake.setPosition(0.55);
    }
    private void goTo(double x, double y, double angle, double speed, double angleSpeed, double distanceDeadzone, double angleDeadzone, boolean velocityControl) {
        control.goTo(x, y, angle, speed, angleSpeed, distanceDeadzone, angleDeadzone, velocityControl);
        while (!control.finished) {
            if (isStopRequested()) control.stop();
            telemetry.addData("Angle: ", pos.angle);
            telemetry.addData("X: ", pos.x);
            telemetry.addData("Y: ", pos.y);
            telemetry.update();
            sleep(10);
        }
    }
    private void setArmPositionWaitFinish(int pos, double speed) {
        robot.leftArm.setTargetPosition(pos);
        robot.rightArm.setTargetPosition(pos);
        robot.leftArm.setPower(speed);
        robot.rightArm.setPower(speed);
        robot.leftArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (!isStopRequested() && robot.leftArm.isBusy()) sleep(10);
    }
    private void setArmPosition(int pos, double speed, int delay) {
        sleep(delay);
        robot.leftArm.setTargetPosition(pos);
        robot.rightArm.setTargetPosition(pos);
        robot.leftArm.setPower(speed);
        robot.rightArm.setPower(speed);
        robot.leftArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    private void goToPole() {
        while (!isStopRequested() && (!(Math.abs(poleDetection.widthError) < 4 && Math.abs(poleDetection.error) < 5 && poleDetection.noPole==0))) {
            robot.fl.setPower(-poleDetection.error * 0.002+poleDetection.widthError * 0.01);
            robot.fr.setPower(poleDetection.error * 0.002+poleDetection.widthError * 0.01);
            robot.bl.setPower(-poleDetection.error * 0.002+poleDetection.widthError * 0.01);
            robot.br.setPower(poleDetection.error * 0.002+poleDetection.widthError * 0.01);
            telemetry.addData("error: ", poleDetection.error);
            telemetry.addData("widthError: ", poleDetection.widthError);
            telemetry.addData("noPole: ", poleDetection.noPole);
            telemetry.update();
            sleep(10);
        }
    }
}
