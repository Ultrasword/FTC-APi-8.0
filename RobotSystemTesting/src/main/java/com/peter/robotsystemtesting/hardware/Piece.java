package com.peter.robotsystemtesting.hardware;

public abstract class Piece {

    private String reference;

    public Piece(String reference){
        this.reference = reference;
    }

    public String getReference() {
        return reference;
    }

}
