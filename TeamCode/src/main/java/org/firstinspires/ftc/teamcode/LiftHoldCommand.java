package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Disabled
public class LiftHoldCommand extends CommandBase {

    private final LiftSubsystem liftSubsystem;

    public LiftHoldCommand(LiftSubsystem liftSubsystem){
        this.liftSubsystem = liftSubsystem;
    }

    @Override
    public void initialize(){
        liftSubsystem.hold();
    }

    @Override
    public void end(boolean interrupted){
        liftSubsystem.stop();
    }


}
