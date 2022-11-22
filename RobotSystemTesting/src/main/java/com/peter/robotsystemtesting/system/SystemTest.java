package com.peter.robotsystemtesting.system;

import com.peter.robotsystemtesting.hardware.DcMotor;
import com.peter.robotsystemtesting.hardware.Piece;
import com.peter.robotsystemtesting.hardware.PieceType;
import com.peter.robotsystemtesting.hardware.Servo;

import java.util.ArrayList;
import java.util.HashMap;

public abstract class SystemTest extends Thread implements Runnable {

    protected boolean active = false;

    // a list of motors and stuff
    public HashMap<PieceType, ArrayList<? extends Piece>> hwPieces = new HashMap<>();

    public void prevInit(){
        hwPieces.put(PieceType.DcMotor, new ArrayList<DcMotor>());
        hwPieces.put(PieceType.Servo, new ArrayList<Servo>());
    }

    public void startOpMode(){
        active = true;
        System.out.println("Press 'K' to exit! (in console)");
    }

    public boolean opModeisActive(){
        // update the stuff

        return active;
    }

    public <T extends Piece> void addHardwarePiece(PieceType type, T piece) {
        if (!hwPieces.containsKey(type))
            new Exception(String.format("The specified type: %s, does not exist! Please use the PieceTypes located in PieceType.java", type.name()));
        // idk man

    }

    // abstract
    public abstract void runOpMode();
    public void run(){
        runOpMode();
    }

}
