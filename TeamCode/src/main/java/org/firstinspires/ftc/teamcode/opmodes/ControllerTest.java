package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.Gamepad;


@TeleOp(name = "ControllerTest")
public class ControllerTest extends LinearOpMode {

    private enum TEST_MODE{
        RUMBLE,
        VIBRATION,
        BLIP,
        EFFECT_1
    }

    Gamepad currentGamepad1 = new Gamepad();
    Gamepad currentGamepad2 = new Gamepad();

    Gamepad previousGamepad1 = new Gamepad();
    Gamepad previousGamepad2 = new Gamepad();


    Gamepad.RumbleEffect effect_1 = new Gamepad.RumbleEffect.Builder()
            .addStep(0, 0.5, 500)
            .addStep(1, 0, 500)
            .addStep(0, 1, 2000)
            .addStep(0.5, 1, 1000)
            .build();

    private TEST_MODE testMode = TEST_MODE.RUMBLE;
    public void mainInit() {

    }

    public void mainLoop() {
        try {
            // Store the gamepad values from the previous loop iteration in
            // previousGamepad1/2 to be used in this loop iteration.
            // This is equivalent to doing this at the end of the previous
            // loop iteration, as it will run in the same order except for
            // the first/last iteration of the loop.
            previousGamepad1.copy(currentGamepad1);
            previousGamepad2.copy(currentGamepad2);

            // Store the gamepad values from this loop iteration in
            // currentGamepad1/2 to be used for the entirety of this loop iteration.
            // This prevents the gamepad values from changing between being
            // used and stored in previousGamepad1/2.
            currentGamepad1.copy(gamepad1);
            currentGamepad2.copy(gamepad2);
        }
        catch (RobotCoreException e) {
            // Swallow the possible exception, it should not happen as
            // currentGamepad1/2 are being copied from valid Gamepads.
        }


        telemetry.addData("Test Mode", testMode);
        telemetry.addData("Controller key pressed", gamepad1.toString());
        telemetry.addData("touchpad finger1", gamepad1.touchpad_finger_1);
        telemetry.addData("touchpad finger2", gamepad1.touchpad_finger_2);
        telemetry.addLine();
        telemetry.addData("touchpad x finger1", gamepad1.touchpad_finger_1_x);
        telemetry.addData("touchpad x finger2", gamepad1.touchpad_finger_2_x);
        telemetry.addData("touchpad y finger1", gamepad1.touchpad_finger_1_y);
        telemetry.addData("touchpad y finger2", gamepad1.touchpad_finger_2_y);
        switch (testMode){
            case RUMBLE:
                if(gamepad1.cross) gamepad1.rumble(100);
                if(gamepad1.circle) gamepad1.rumble(0.5,1,100);
                if(gamepad1.square) gamepad1.rumble(0,1,100);
                if(gamepad1.triangle) gamepad1.rumble(0.1,1,100);
                if(gamepad1.cross) gamepad1.setLedColor(1.0,1.0,1.0, 100);
                if(gamepad1.circle) gamepad1.setLedColor(1.0,0,0, 100);
                if(gamepad1.square) gamepad1.setLedColor(0,1,0, 100);
                if(gamepad1.triangle) gamepad1.setLedColor(0,0,1, 100);
                break;
            case BLIP:
                if(gamepad1.cross) gamepad1.rumbleBlips(10);
                break;
            case EFFECT_1:
                if(gamepad1.cross) gamepad1.runRumbleEffect(effect_1);
                break;
        }
        //toggle between test modes
        if(currentGamepad1.dpad_up && !previousGamepad1.dpad_up)
            switch (testMode){
                case RUMBLE:
                    testMode = TEST_MODE.BLIP;
                    break;
                case BLIP:
                    testMode = TEST_MODE.EFFECT_1;
                    break;
                case EFFECT_1:
                    testMode = TEST_MODE.RUMBLE;
                    break;
            }
        telemetry.update();
    }

    public void runOpMode(){

        waitForStart();

        while(opModeIsActive()){
            mainLoop();
        }
    }
}