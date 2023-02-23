package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.wrappers.DetectConeDisplay;
import org.firstinspires.ftc.teamcode.wrappers.DisplayVision;
import org.firstinspires.ftc.teamcode.wrappers.Vision;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name = "VisionTest")
public class VisionTest extends LinearOpMode {
    private FtcDashboard dashboard = FtcDashboard.getInstance();
    private OpenCvCamera camera;
    private TelemetryPacket telemetry = new TelemetryPacket();
    @Override
    public void runOpMode() throws InterruptedException {
        DetectConeDisplay detection = new DetectConeDisplay(true);
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Camera"), cameraMonitorViewId);
        camera.setPipeline(detection);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
                dashboard.startCameraStream(camera, 30);
            }
            @Override
            public void onError(int errorCode) {
            }
        });
        while (!isStarted()) {
            telemetry.put("hue",detection.hsvColor[0]);
            telemetry.put("saturation",detection.hsvColor[1]);
            telemetry.put("value",detection.hsvColor[2]);
            dashboard.sendTelemetryPacket(telemetry);
            sleep(200);
        }
        camera.closeCameraDeviceAsync(() -> {
            dashboard.stopCameraStream();
        });
    }
}