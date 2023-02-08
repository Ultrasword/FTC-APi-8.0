package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.ftccommon.FtcAdvancedRCSettingsActivity;
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
import org.openftc.easyopencv.OpenCvInternalCamera2;

@Config
@Autonomous(name="VisionTest")
public class VisionTest extends LinearOpMode {
    private Vision sleeveDetection;
    private MecanumChassis robot;
    private DetectPoleDisplay poleDetection;
    private WebcamName webcamName;
    private OpenCvCamera camera;

    public static double POWERTEST = 0.0;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new MecanumChassis(hardwareMap);
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        // webcam
        camera = OpenCvCameraFactory.getInstance().createInternalCamera2(OpenCvInternalCamera2.CameraDirection.BACK, cameraMonitorViewId);
        webcamName = hardwareMap.get(WebcamName.class, "Camera");

        sleeveDetection = new Vision();
        poleDetection = new DetectPoleDisplay();
        // pipeline
        camera.setPipeline(sleeveDetection);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(320,240, OpenCvCameraRotation.SIDEWAYS_RIGHT);
            }
            @Override
            public void onError(int errorCode) {}
        });

        FtcDashboard.getInstance().startCameraStream(camera, 60);
        while (!isStarted()) {
            telemetry.addData("route: ", sleeveDetection.route);
            telemetry.update();
        }

        waitForStart();
        // start
        camera.setPipeline(poleDetection);
        goToPole();
    }
    private void goToPole() {
        while (!isStopRequested() && (!(Math.abs(poleDetection.widthError) < 4 && Math.abs(poleDetection.error) < 5))) {
            robot.fl.setPower(-poleDetection.error * 0.002+poleDetection.widthError * 0.01);
            robot.fr.setPower(poleDetection.error * 0.002+poleDetection.widthError * 0.01);
            robot.bl.setPower(-poleDetection.error * 0.002+poleDetection.widthError * 0.01);
            robot.br.setPower(poleDetection.error * 0.002+poleDetection.widthError * 0.01);
            telemetry.addData("error: ", poleDetection.error);
            telemetry.addData("widthError: ", poleDetection.widthError);
//            telemetry.addData("noPole: ", poleDetection.noPole);
            telemetry.update();
            sleep(10);
        }
    }
}