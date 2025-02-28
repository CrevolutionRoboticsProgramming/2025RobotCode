package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.ElbowPivot.ElbowPivot;
import frc.robot.ElbowPivot.commands.ElbowPivotCommands;
import frc.robot.ElbowPivot.commands.SetElbowPivotAngle;
import frc.robot.WristPivot.WristPivot;
import frc.robot.WristPivot.commands.SetWristPivotAngle;
import frc.robot.WristPivot.commands.WristPivotCommands;
import frc.robot.algaeflywheel.AlgaeFlyWheel;
import frc.robot.algaeflywheel.commands.AlgaeFlyWheelCommands;

public class RobotCommands {
    public static Command coralL1() {
        return new SequentialCommandGroup(
            new ParallelCommandGroup(
                WristPivotCommands.setWristPivotAngle(SetWristPivotAngle.Preset.kCoralL1),
                ElbowPivotCommands.setElbowPivotAngle(SetElbowPivotAngle.Preset.kCoralL1)
            )
        );
    }

    public static Command primeShoot() {
        return new SequentialCommandGroup(
            new WaitCommand(0.5),
            AlgaeFlyWheelCommands.setAngularVelocity(
                () -> AlgaeFlyWheel.Settings.kMaxAngularVelocity.times(0.8)
            )
        );
    }

}
