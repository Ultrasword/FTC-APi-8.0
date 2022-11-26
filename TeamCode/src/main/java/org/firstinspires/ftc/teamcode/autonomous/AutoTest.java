package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.system.OpModeSGlobals;
import org.firstinspires.ftc.teamcode.system.Vision;
import org.firstinspires.ftc.teamcode.system.Chassis;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name="Auto Camera Test")
public class AutoTest extends LinearOpMode {
    Vision sleeveDetection = new Vision();
    OpenCvCamera camera;
    String webcamName = "Webcam 1";

    @Override
    public void runOpMode() throws InterruptedException {
        OpModeSGlobals.initOpMode(this);
        Chassis chassis = new Chassis();
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//        camera = OpenCvCameraFactory.getInstance().createInternalCamera2(OpenCvInternalCamera2.CameraDirection.BACK, cameraMonitorViewId);
        WebcamName webcamName = hardwareMap.get(WebcamName.class,"camera");
        //OpenCvCamera camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName); without live preview
        OpenCvCamera camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);

        sleeveDetection = new Vision();
        camera.setPipeline(sleeveDetection);

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(320,240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {telemetry.addData("Cringe error", "");}
        });

        while (!isStarted()) {
            telemetry.addData("POSITION: ", sleeveDetection.getPosition());
            telemetry.update();
        }


        // move now
        waitForStart();

        while(opModeIsActive()){
            if(sleeveDetection.getPosition() == Vision.ParkingPosition.LEFT){

            }
            else if(sleeveDetection.getPosition() == Vision.ParkingPosition.RIGHT){

            }
            else if(sleeveDetection.getPosition() == Vision.ParkingPosition.CENTER){
                chassis.goStraight(6);
            }
            sleep(1000);
        }

    }
}
