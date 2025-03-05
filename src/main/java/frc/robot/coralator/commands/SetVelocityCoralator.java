package frc.robot.coralator.commands;

import java.util.function.Supplier;

import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.coralator.CoralRollerSubsystem;
import frc.robot.operator.OperatorXbox;

public class SetVelocityCoralator extends Command{
    double volts;
    public SetVelocityCoralator(double volts) {
        this.volts = volts;
        addRequirements(CoralRollerSubsystem.getInstance());
    }

    @Override
    public void initialize() {
        CoralRollerSubsystem.getInstance().setVoltage(volts);
    }
        
}
