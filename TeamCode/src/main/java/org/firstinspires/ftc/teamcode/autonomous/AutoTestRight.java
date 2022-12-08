package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.wrappers.Controller;
import org.firstinspires.ftc.teamcode.wrappers.MecanumChassis;
import org.firstinspires.ftc.teamcode.wrappers.Position;
import org.firstinspires.ftc.teamcode.wrappers.Vision;
import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name="autoright")
public class AutoTestRight extends LinearOpMode {
    private MecanumChassis robot;
    private Position pos;
    private Controller control;
    private Vision sleeveDetection;
    private WebcamName webcamName;
    private OpenCvCamera camera;
    private String route;

    private double MPerFoot = 0.3048, MatSize = MPerFoot*2;

    private int coneHeightDif = 10, coneCount = 5;
    private double armSpeed = 0.4, movementSpeed = 0.8;
    private int armTop = 520, armDrop = 350, armBottom = 10, armMedHeight = 100;

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

        // ----------- auto portion ------------ //
        // step 1: cycle cone pickup and place from left side
        // twice
        resetCycle();
        pickupLeftConePivot();
        resetCycle();
        pickupLeftConePivot();
        resetCycle();
        // go pickup --> this cone goes on the parking junction
        goTo(-MatSize, MatSize*2, -90, 0.6, 0.2, 0.01, 0.02, true);
        closeIntake();
        setArmPosition(armMedHeight, 0.3);
        // rotation towards junction
        goTo(0, MatSize*2, 46, movementSpeed, 0.3, 0.01, 0.02, true);
        setArmPosition(armTop, armSpeed);
        // arm position --



        // ----------- parking ------------ //
        switch (route) {
            case "LEFT":
                setArmPosition(520, armSpeed);
                goTo(-0.3,0.6,45, movementSpeed, 50,0.04,2,true);
                goTo(-0.3+0.06,0.6+0.03,45,0.4, 50,0.04,2,true);
                setArmPosition(350, armSpeed);
                goTo(-0.3,0.6,0,0.4, 50,0.04,2,true);
                setArmPosition(0, armSpeed);
                break;
            case "CENTER":
                setArmPosition(520, armSpeed);
                goTo(0,0.9,45,1.2, 50,0.04,2,true);
                goTo(0,0.6,45,1.2, 50,0.04,2,true);
                goTo(0+0.06,0.6+0.03,45,0.4, 50,0.04,2,true);
                setArmPosition(350, armSpeed);
                openIntake();
                goTo(0,0.6,0,0.4, 50,0.04,2,true);
                setArmPosition(0, armSpeed);
                break;

            case "RIGHT":
                setArmPosition(520, armSpeed);
                goTo(0.3,0.9,45, movementSpeed, 50,0.04,2,true);
                goTo(0.3,0.6,45, movementSpeed, 50,0.04,2,true);
                goTo(0.3+0.06,0.6+0.03,45,0.4, 50,0.04,2,true);
                setArmPosition(350, armSpeed);
                openIntake();
                goTo(0.3,0.6,0,0.4, 50,0.04,2,true);
                setArmPosition(0, armSpeed);
                break;
            default:
                telemetry.addData("OH SHIT!","WE FUCKED UP!");
                telemetry.update();
                sleep(2000);
                break;
        }
    }

    private void pickupLeftConePivot(){
        // moves to pivot position, move to left cone pickup location, move to junction and raise and drop
        openIntake();
        setArmPosition(coneHeightDif * coneCount, armSpeed);
        coneCount--;
        goTo(-MatSize*0.8, MatSize*2, -90, 0.6, 0.2, 0.01, 0.2, true);
        goTo(-MatSize, MatSize*2, -90, 0.6, 0.2, 0.01, 0.02, true);
        closeIntake();
        setArmPosition(armMedHeight, 0.3);
        // rotation towards junction
        goTo(0, MatSize*2, 46, movementSpeed, 0.3, 0.01, 0.02, true);
        setArmPosition(armTop, armSpeed);
        goTo(MatSize*0.2, MatSize*2.2, 46, movementSpeed, 0.3, 0.01, 0.02, true);
        sleep(200);
        // drop and release
        setArmPosition(armDrop, armSpeed);
        openIntake();
        resetCycle();
        setArmPosition(0, armSpeed);
        // final position will be at pivot -- arm down -- -90deg
    }

    private void resetCycle(){
        goTo(0, MatSize*2, -90, 0.6, 0.2, 0.01, 0.02, true);
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
