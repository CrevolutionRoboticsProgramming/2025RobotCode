package frc.robot.coralArm.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.coralArm.CoralSubsystem;

public class CoralArmCommands {
    public static Command setCoralArmState(CoralSubsystem.State state) {
        return new SetTargetStateCoral(state);
    }
}
