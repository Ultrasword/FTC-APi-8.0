package org.firstinspires.ftc.teamcode.system;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public abstract class System {

    public Telemetry telemetry;

    public System(Telemetry telemetry){
        this.telemetry = telemetry;
    }

    public abstract void update();

}
