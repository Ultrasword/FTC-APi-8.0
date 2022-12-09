package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.wrappers.DetectPole;
import org.firstinspires.ftc.teamcode.wrappers.DetectPoleDisplay;
import org.firstinspires.ftc.teamcode.wrappers.DisplayVision;
import org.firstinspires.ftc.teamcode.wrappers.MecanumChassis;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.firstinspires.ftc.teamcode.wrappers.Vision;

@Autonomous(name="VisionTest")
public class VisionTest extends LinearOpMode {
    private Vision sleeveDetection;
    private MecanumChassis robot;
    private DetectPole poleDetection;
    private WebcamName webcamName;
    private OpenCvCamera camera;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new MecanumChassis(hardwareMap);
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcamName = hardwareMap.get(WebcamName.class, "Camera");
        camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
        sleeveDetection = new Vision();
        poleDetection = new DetectPole();
        camera.setPipeline(poleDetection);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(320,240, OpenCvCameraRotation.SIDEWAYS_RIGHT);
            }
            @Override
            public void onError(int errorCode) {}
        });

        while (!isStarted()) {
            telemetry.addData("error: ", poleDetection.error);
            telemetry.addData("width error: ", poleDetection.widthError);
            telemetry.update();
        }
        waitForStart();
        camera.setPipeline(poleDetection);
        goToPole();
    }
    private void goToPole() {
        while (!isStopRequested() && (!(Math.abs(poleDetection.widthError) < 2 && Math.abs(poleDetection.error) < 3 && poleDetection.noPole<4))) {
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