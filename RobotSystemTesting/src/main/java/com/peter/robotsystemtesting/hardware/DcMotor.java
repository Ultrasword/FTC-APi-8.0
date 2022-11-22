package com.peter.robotsystemtesting.hardware;

public class DcMotor extends Piece {

    private String name;
    private double power;

    private long ticks;

    public DcMotor(String reference){
        super(reference);
        this.name = reference;
    }

    public void update(){

    }

}
