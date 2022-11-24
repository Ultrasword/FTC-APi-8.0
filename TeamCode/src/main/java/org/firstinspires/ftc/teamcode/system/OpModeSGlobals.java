package org.firstinspires.ftc.teamcode.system;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.checkerframework.checker.index.qual.LTEqLengthOf;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.wrappers.OpModeWrapper;

public class OpModeSGlobals {

    public static Telemetry telemetry;
    public static LinearOpMode opmode;
    public static Gamepad gamepad1, gamepad2;
    public static HardwareMap hwMap;

    public static double DRIVETRAIN_WHEEL_DIAMETER_INCHES = 3.0,
                        ARMDISTANCE_THING = 0.0;

    public static void initOpMode(LinearOpMode mode){
        opmode = mode;
        telemetry = mode.telemetry;
        gamepad1 = mode.gamepad1;
        gamepad2 = mode.gamepad2;
        hwMap = mode.hardwareMap;
    }

    public static void initOpMode(OpModeWrapper mode){
        opmode = mode;
        telemetry = mode.telemetry;
        gamepad1 = mode.gamepad1;
        gamepad2 = mode.gamepad2;
        hwMap = mode.hardwareMap;
    }

}
