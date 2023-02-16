package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
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

    public static double COEF = 0.002, ERRORCOEF = 0.01;

    /*
        NOTES FOR TMR OR SMTH
        1. camera oritneted so that final position of junction == left side of screen
            a. so we dont need to worry abt orientation or stuff
            b. prerequisites for it to work!
                i. must be heading towards junction initially
            c. code just orients heading to face the junction THAT IS ALREADY in the vision scope
                of robot
        2. tape camera to good orientation :)
            a. or zip tie or smth

     */

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new MecanumChassis(hardwareMap);
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcamName = hardwareMap.get(WebcamName.class, "Camera");
//        camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
        camera = OpenCvCameraFactory.getInstance().createInternalCamera2(OpenCvInternalCamera2.CameraDirection.BACK, cameraMonitorViewId);
        sleeveDetection = new Vision();
        poleDetection = new DetectPoleDisplay();

        // set pipeline
        camera.setPipeline(poleDetection);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(320,240, OpenCvCameraRotation.SIDEWAYS_RIGHT);
            }
            @Override
            public void onError(int errorCode) {}
        });
        FtcDashboard.getInstance().startCameraStream(camera, 30);

        // pause
        while (!isStarted()) {
            telemetry.addData("error: ", poleDetection.error);
            telemetry.addData("width error: ", poleDetection.widthError);
            telemetry.update();
        }
        waitForStart();
        camera.setPipeline(poleDetection);
        // go to pole
        goToPole();
    }
    private void goToPole() {
        while (!isStopRequested() && (!(Math.abs(poleDetection.widthError) < 4 && Math.abs(poleDetection.error) < 5))) {
            robot.fl.setPower(-poleDetection.error * COEF + poleDetection.widthError * ERRORCOEF);
            robot.fr.setPower(poleDetection.error * COEF + poleDetection.widthError * ERRORCOEF);
            robot.bl.setPower(-poleDetection.error * COEF + poleDetection.widthError * ERRORCOEF);
            robot.br.setPower(poleDetection.error * COEF + poleDetection.widthError * ERRORCOEF);
            telemetry.addData("error: ", poleDetection.error);
            telemetry.addData("widthError: ", poleDetection.widthError);
//            telemetry.addData("noPole: ", poleDetection.noPole);
            telemetry.update();
            sleep(10);
        }
    }
}