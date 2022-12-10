package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.wrappers.Controller;
import org.firstinspires.ftc.teamcode.wrappers.DetectPole;
import org.firstinspires.ftc.teamcode.wrappers.DetectPoleDisplay;
import org.firstinspires.ftc.teamcode.wrappers.MecanumChassis;
import org.firstinspires.ftc.teamcode.wrappers.Position;
import org.firstinspires.ftc.teamcode.wrappers.Vision;
import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name="autonovisionright")
public class AutoTestRight extends LinearOpMode {
    private MecanumChassis robot;
    private Position pos;
    private Controller control;
    private Vision sleeveDetection;
    private DetectPole poleDetection;
    private WebcamName webcamName;
    private OpenCvCamera camera;
    private String route;

    private double MPerFoot = 0.3048, MatSize = MPerFoot*2;

    private int coneHeightDif = 20, coneCount = 10;
    private double armSpeed = 0.4, movementSpeed = 0.6, angleSpeed = 50;
    private int armTop = 520, armDrop = 350, armBottom = 10, armMedHeight = 100;
    private double angleDeadZone = 2, moveDeadZone = 0.04;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new MecanumChassis(hardwareMap);
        pos = new Position(robot);
        control = new Controller(robot, pos);
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcamName = hardwareMap.get(WebcamName.class, "Camera");
        camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
        sleeveDetection = new Vision();
        poleDetection = new DetectPole();
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

        /*
        Initial Strategy:
            - we are moving to pivot position, then spinning

        ConeDetection Strategy:
            - move to center and spin towards edge --> the move towards cone using cone detection

        !!!WE ARE USING INITIAL STRATEGY!!!

         */
        // ----------- auto/cycle portion ------------ //
        // step 1: pick up preload cone --> place on high junction
        // 2 mats forward, and turn left 45deg
        closeIntake();
        setArmPosition(armMedHeight, armSpeed, 200);
        toPivotLocation(-45);
        setArmPosition(armTop, armSpeed, 0);
        goToDefault(-MatSize*0.2, MatSize*2.2, -45);//
        setArmPositionWaitFinish(armMedHeight, armSpeed);
        openIntake();
        toPivotLocation(90);

        // step 2: cycle cone pickup and placing
        for(int i = 0; i < 2; i++){
            pickupSideConeThenToPivot(-45, armTop);
            // REPLACE with darren junction align
            goToDefault(-MatSize * 0.2, MatSize * 2.2, -45);
            // ----
            setArmPositionWaitFinish(armMedHeight, armSpeed);
            openIntake();
            // moving down :)
            toPivotLocation(90);
        } // end
        // ------------- final segment --------------- //
        // step 3: pick up final cone
        pickupSideConeThenToPivot(90, armMedHeight);
        // move back to designated junction depending on distance - find move back distance
        double dcoef = route.equals("LEFT") ? -2 : (route.equals("CENTER") ? -1 : 0.7);
        goToDefault(MatSize * dcoef, MatSize*2, 90);

        // step 4: place cone
        // go to cone position
        dcoef = route.equals("LEFT") ? -2 : (route.equals("CENTER") ? -1 : 0);
        setArmPosition(armMedHeight, armSpeed, 0);
        setArmPositionWaitFinish(armTop, armSpeed);
        goToDefault(MatSize * (dcoef-0.2), MatSize*2, -135);
        // placing cone
        setArmPositionWaitFinish(armMedHeight, armSpeed);
        openIntake();
        goToDefault(MatSize*(dcoef+1), MatSize*2, -180);


        setArmPosition(0, armSpeed, 0);
    }

    // ----------- stick stuff -------------- //
    private void toPivotLocation(double angle){
        goToDefault(0, MatSize*2, angle);
    }
    private void pickupSideConeThenToPivot(double finalAngle, int finalArmHeight){
        // moves to pivot position, move to left cone pickup location, move to junction and raise and drop
        setArmPosition(coneHeightDif * coneCount, armSpeed, 0);
        coneCount--;
        // now move to side -- not completely side
        goToDefault(MatSize*0.6, MatSize*2, 90);
        // pickup cone -- and move to side
        goToDefault(MatSize*0.8, MatSize*2, 90);
        closeIntake();
        // move arm up while moving back
        setArmPosition(armMedHeight, 0.3, 500);
        goToDefault(MatSize*0.5, MatSize*2, 90);
        toPivotLocation(finalAngle);
        setArmPosition(finalArmHeight, 0.3, 0);
    }

    // ---------- end ----------- //

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
