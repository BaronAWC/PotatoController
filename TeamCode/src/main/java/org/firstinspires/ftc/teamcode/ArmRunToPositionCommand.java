package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandBase;

public class ArmRunToPositionCommand extends CommandBase {

    private final ArmSubsystem armSubsystem;
    private final int position;
    private final double power;

    public ArmRunToPositionCommand(ArmSubsystem armSubsystem, int position, double power){
        this.armSubsystem = armSubsystem;
        this.position = position;
        this.power = power;
    }

    @Override
    public void initialize(){
        armSubsystem.runToPosition(position, power);
    }

    @Override
    public boolean isFinished(){
        return armSubsystem.isFinished();
    }
}
