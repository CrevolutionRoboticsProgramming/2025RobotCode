package frc.robot.coralator.commands;

import java.util.function.Supplier;

import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.coralator.Coralator;
import frc.robot.operator.OperatorXbox;

public class SetVelocityCoralator extends Command{
    private final Coralator mCoralator;
    private final Supplier<Rotation2d> velocitySupplier;
    private final InvertedValue kInvertedValue;

    SetVelocityCoralator(Supplier<Rotation2d> velocitySupplier, InvertedValue kInvertedValue) {
        mCoralator = Coralator.getInstance();
        this.velocitySupplier = velocitySupplier;
        this.kInvertedValue = kInvertedValue;
    }

    @Override
    public void execute() {
        mCoralator.setVelocity(velocitySupplier.get(), kInvertedValue);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean isInterrupted) {
        mCoralator.setVelocity(Rotation2d.fromDegrees(0), kInvertedValue);
    }
}
