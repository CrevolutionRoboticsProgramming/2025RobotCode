package frc.robot.coralArm.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.coralArm.CoralSubsystem;
import frc.robot.coralArm.CoralSubsystem.State;

public class SetTargetStateCoral extends Command{

    private State state;
    public SetTargetStateCoral(State state) {
        this.state = state;
        addRequirements(CoralSubsystem.getInstance());
    }

    @Override
    public void initialize() {
        CoralSubsystem.getInstance().setTargetState(state);
    }

}
