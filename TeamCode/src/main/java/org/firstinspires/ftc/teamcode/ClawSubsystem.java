package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Servo;

public class ClawSubsystem extends SubsystemBase {

    public static final double OPEN = 0.14, CLOSED = 0;
    private final Servo claw;

    public ClawSubsystem(Servo claw){
        this.claw = claw;
        claw.setDirection(Servo.Direction.FORWARD);
    }

    public void open(){
        claw.setPosition(OPEN);
    }

    public void close(){
        claw.setPosition(CLOSED);
    }

    public void stop(){
        claw.setPosition(claw.getPosition());
    }
}
