package frc.robot.algaeflywheel.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.algaeflywheel.AlgaeRoller;
import frc.robot.operator.OperatorXbox;

public class SetRollerVelAutoShoot extends Command{
    private final AlgaeRoller algaeSystem;
    private final Supplier<Rotation2d> velocitySupplier;

    private final Rotation2d kAllowedError = Rotation2d.fromRotations(5); // 300 RPM


    SetRollerVelAutoShoot(Supplier<Rotation2d> velocitySupplier) {
        algaeSystem = AlgaeRoller.getInstance();
        this.velocitySupplier = velocitySupplier;
    }

    @Override
    public void execute() {
        final Rotation2d velFlywheel = velocitySupplier.get();
        final Rotation2d leftVel = algaeSystem.geLeftVelocity();
        final Rotation2d rightVel = algaeSystem.getRightVelocity();

        algaeSystem.setFlywheelVelocity(velFlywheel);

        SmartDashboard.putBoolean("Shooter Ready (left)", 
            (Math.abs(velFlywheel.getRotations()) - (Math.abs(leftVel.getRotations()))) < kAllowedError.getRotations());
        SmartDashboard.putBoolean("Shooter Ready (right)", 
            (Math.abs(velFlywheel.getRotations()) - (Math.abs(rightVel.getRotations()))) < kAllowedError.getRotations());
    
        var leftAtVel = 
            (Math.abs(velFlywheel.getRotations()) - (Math.abs(leftVel.getRotations()))) < kAllowedError.getRotations();
        var rightAtVel = 
            (Math.abs(velFlywheel.getRotations()) - (Math.abs(rightVel.getRotations()))) < kAllowedError.getRotations();
        
        if (leftAtVel || rightAtVel) {
            algaeSystem.setIndexerVoltage(12);
        } else {
            algaeSystem.setIndexerVoltage(-6);
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean isInterrupted) {
        
    }
}
