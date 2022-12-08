package org.firstinspires.ftc.teamcode.autonomous;

import android.util.Pair;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.wrappers.Controller;
import org.firstinspires.ftc.teamcode.wrappers.MecanumChassis;
import org.firstinspires.ftc.teamcode.wrappers.Position;
import org.firstinspires.ftc.teamcode.wrappers.Vision;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name="auton")
public class AutoTest extends LinearOpMode {
    private MecanumChassis robot;
    private Position pos;
    private Controller control;
    private Vision sleeveDetection;
    private WebcamName webcamName;
    private OpenCvCamera camera;
    private String route;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new MecanumChassis(hardwareMap);
        pos = new Position(robot);
        control = new Controller(robot, pos);
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcamName = hardwareMap.get(WebcamName.class, "Camera");
        camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
        sleeveDetection = new Vision();
        camera.setPipeline(sleeveDetection);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(320,240, OpenCvCameraRotation.SIDEWAYS_RIGHT);
            }
            @Override
            public void onError(int errorCode) {}
        });
        while (!isStarted()) {
            telemetry.addData("route: ", sleeveDetection.route);
            telemetry.update();
        }
        route = sleeveDetection.route;
        waitForStart();
        closeIntake();

        double armSpeed = 0.2;
        double movementSpeed = 1.2;
        MovementWrapper HJsetup = new MovementWrapper(0.0,0.9,45);

        int armTop = 520;
        int armDrop = 350;
        int armBottom = 10;

        //Move to HJ
        setArmPositionTiming(armTop, armSpeed,1000);
        goTo(HJsetup.x, HJsetup.y, HJsetup.angle, movementSpeed,50,0.04,2,true);

        //Drop cone on HJ
        goTo(0.06,0.93,45,0.8,50,0.02,2,true);
        setArmPositionWait(armDrop, armSpeed);
        openIntake();

        //Move to side cones
        goTo(HJsetup.x, HJsetup.y, HJsetup.angle, 0.8,50,0.04,2,true);
        setArmPosition(armBottom, armSpeed);
        goTo(-0.4,0.86,-90,0.8,150,0.02,2,true);


        // TODO This is start of HJ cycle
        closeIntake();
        setArmPositionWait(60, armSpeed);

        //Move to HJ
        setArmPosition(armTop, armSpeed);
        goTo(0, 0.9, 45, movementSpeed, 50, 0.04, 2, true);

        //Drop cone on HJ
        goTo(0.06,0.93,45,0.8,50,0.02,2,true);
        setArmPositionWait(350,armSpeed);
        openIntake();

        //Move to side cones
        goTo(0,0.9,45,0.8,50,0.04,2,true);
        setArmPosition(armBottom, armSpeed);
        goTo(-0.4,0.86,-90,0.8,150,0.02,2,true);
        // TODO This is end of HJ cycle

        closeIntake();
        setArmPositionWait(60, armSpeed);

        switch (route) {
            case "LEFT":
                setArmPosition(520, armSpeed);
                goTo(-0.3,0.6,45, movementSpeed, 50,0.04,2,true);
                goTo(-0.3+0.06,0.6+0.03,45,0.4, 50,0.04,2,true);
                setArmPositionWait(350, armSpeed);
                goTo(-0.3,0.6,0,0.4, 50,0.04,2,true);
                setArmPositionWait(0, armSpeed);
                break;
            case "CENTER":
                setArmPosition(520, armSpeed);
                goTo(0,0.9,45,1.2, 50,0.04,2,true);
                goTo(0,0.6,45,1.2, 50,0.04,2,true);
                goTo(0+0.06,0.6+0.03,45,0.4, 50,0.04,2,true);
                setArmPositionWait(350, armSpeed);
                openIntake();
                goTo(0,0.6,0,0.4, 50,0.04,2,true);
                setArmPositionWait(0, armSpeed);
                break;

            case "RIGHT":
                setArmPosition(520, armSpeed);
                goTo(0.3,0.9,45, movementSpeed, 50,0.04,2,true);
                goTo(0.3,0.6,45, movementSpeed, 50,0.04,2,true);
                goTo(0.3+0.06,0.6+0.03,45,0.4, 50,0.04,2,true);
                setArmPosition(350, armSpeed);
                openIntake();
                goTo(0.3,0.6,0,0.4, 50,0.04,2,true);
                setArmPositionWait(0, armSpeed);
                break;
            default:
                telemetry.addData("OH SHIT!","WE FUCKED UP!");
                telemetry.update();
                sleep(2000);
                break;
        }
    }
    private void closeIntake() {
        robot.intake.setPosition(0.75);
    }
    private void openIntake() {
        robot.intake.setPosition(0.55);
    }
    private void goTo(double x, double y, double angle, double speed, double angleSpeed, double distanceDeadzone, double angleDeadzone, boolean velocityControl) {
        control.goTo(x, y, angle, speed, angleSpeed, distanceDeadzone, angleDeadzone, velocityControl);
        while (!control.finished) {
            telemetry.addData("Angle: ", pos.angle);
            telemetry.addData("X: ", pos.x);
            telemetry.addData("Y: ", pos.y);
            telemetry.update();
            sleep(10);
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
    private void setArmPositionWait(int pos, double speed) {
        robot.leftArm.setTargetPosition(pos);
        robot.rightArm.setTargetPosition(pos);
        robot.leftArm.setPower(speed);
        robot.rightArm.setPower(speed);
        robot.leftArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (robot.leftArm.isBusy()) sleep(10);
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


class MovementWrapper {
    public double x;
    public double y;
    public double angle;

    public MovementWrapper(double x, double y, double angle) {
        this.x = x;
        this.y = y;
        this.angle = angle;
    }
}