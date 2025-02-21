package frc.robot.algaeflywheel.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.algaeflywheel.AlgaeFlyWheel;
import frc.robot.operator.OperatorXbox;

import java.util.function.Supplier;

import com.ctre.phoenix6.signals.InvertedValue;

public class SetVelocityAlgaeFlyWheel extends Command {
    private final AlgaeFlyWheel flywheel;
    private final Supplier<Rotation2d> leftVelocitySupplier, rightVelocitySupplier;
    private final InvertedValue kInvertedValue;

    private final Rotation2d kAllowedError = Rotation2d.fromRotations(5); // 300 RPM

    SetVelocityAlgaeFlyWheel(Supplier<Rotation2d> leftVelocitySupplier, Supplier<Rotation2d> rightVelocitySupplier, InvertedValue kInvertedValue) {
        flywheel = AlgaeFlyWheel.getInstance();
        this.leftVelocitySupplier = leftVelocitySupplier;
        this.rightVelocitySupplier = rightVelocitySupplier;
        this.kInvertedValue = kInvertedValue;
    }

    SetVelocityAlgaeFlyWheel(Supplier<Rotation2d> velocitySupplier, InvertedValue kInvertedValue) {
        this(velocitySupplier, velocitySupplier, kInvertedValue);
    }

    @Override
    public void execute() {
        final var leftVel = leftVelocitySupplier.get();
        final var rightVel = rightVelocitySupplier.get();
        flywheel.setRightFlywheelVelocity(leftVel, kInvertedValue);
        flywheel.setLeftFlywheelVelocity(rightVel, kInvertedValue);

        SmartDashboard.putBoolean("Shooter Ready (left)", (Math.abs(leftVel.getRotations()) - (Math.abs(flywheel.getLeftFlywheelVelocity().getRotations()))) < kAllowedError.getRotations());
        SmartDashboard.putBoolean("Shooter Ready (right)", (Math.abs(rightVel.getRotations()) - (Math.abs(flywheel.getRightFlywheelVelocity().getRotations()))) < kAllowedError.getRotations());
    
        var leftAtVel = (Math.abs(leftVel.getRotations()) - (Math.abs(flywheel.getLeftFlywheelVelocity().getRotations()))) < kAllowedError.getRotations();
        var rightAtVel = (Math.abs(rightVel.getRotations()) - (Math.abs(flywheel.getRightFlywheelVelocity().getRotations()))) < kAllowedError.getRotations();
        if (leftAtVel || rightAtVel) {
            OperatorXbox.getInstance().controller.getHID().setRumble(RumbleType.kBothRumble, 1);
        } else {
            OperatorXbox.getInstance().controller.getHID().setRumble(RumbleType.kBothRumble, 0);
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean isInterrupted) {
        flywheel.setRightFlywheelVelocity(Rotation2d.fromDegrees(0), kInvertedValue);
        flywheel.setLeftFlywheelVelocity(Rotation2d.fromDegrees(0), kInvertedValue);
    }
}
