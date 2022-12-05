package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.firstinspires.ftc.teamcode.wrappers.Vision;

@Autonomous(name="Vision funny")
public class VisionTest extends LinearOpMode {

    private Vision sleeveDetection;
    private WebcamName webcamName;
    private OpenCvCamera camera;

    @Override
    public void runOpMode() throws InterruptedException {
        webcamName = hardwareMap.get(WebcamName.class, "Camera");
        camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName);
        sleeveDetection = new Vision();
        camera.setPipeline(sleeveDetection);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(320,240, OpenCvCameraRotation.UPRIGHT);
            }
            @Override
            public void onError(int errorCode) {}
        });

        while (!isStarted()) {
            telemetry.addData("route: ", sleeveDetection.route);
            telemetry.update();
        }
        waitForStart();
    }
}