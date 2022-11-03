package org.firstinspires.ftc.teamcode.system;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.checkerframework.checker.index.qual.LTEqLengthOf;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class OpModeSGlobals {

    public static Telemetry telemetry;

    public static void initOpMode(LinearOpMode mode){
        telemetry = mode.telemetry;
    }

}
