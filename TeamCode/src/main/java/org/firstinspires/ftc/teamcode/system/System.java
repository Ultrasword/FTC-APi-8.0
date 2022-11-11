package org.firstinspires.ftc.teamcode.system;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public abstract class System {

    public System() {}

    public abstract void update();

    public void updateTelemetry(Telemetry telemetry){

    }

}
