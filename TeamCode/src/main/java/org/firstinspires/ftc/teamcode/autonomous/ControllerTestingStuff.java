package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Controller Testing Stuff")
public class ControllerTestingStuff extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {


        telemetry.addData("Start", "");
        telemetry.update();

        waitForStart();

        double col[] = {0, 0.2, 0.1};


        while(opModeIsActive()){

            // check if touchpad thing works
            double valx = gamepad1.touchpad_finger_1_x;
            double valy = gamepad1.touchpad_finger_1_y;

            updateColor(col, 0, 0.01);
            updateColor(col, 1, 0.03);
            updateColor(col, 2, 0.04);
            gamepad1.setLedColor(col[0], col[1], col[2], 100);
             if(gamepad1.cross) gamepad1.rumble(100);

            telemetry.addData("Touchpad data: ", String.format("%.2f, %.2f", valx, valy));

            telemetry.update();

            sleep(100);

        }
    }

    private void updateColor(double col[], int index, double inc){
        col[index] += inc; if (col[index] > 1.0) col[index] = 0.0;
    }
}
