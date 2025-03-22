package frc.robot.rushinator;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.algaeflywheel.AlgaeRoller;


public class RushinatorRollers extends SubsystemBase {
    public static class Settings {
        static final int kTopID = 26;

        // RPS
        static final Rotation2d kMaxAngluarVelocity = Rotation2d.fromRotations(6000 / 60);
    }

    private static RushinatorRollers mInstance;
    private final TalonFX mTalonWheel;

    private RushinatorRollers() {
        mTalonWheel = new TalonFX(Settings.kTopID);
    }

    public static RushinatorRollers getInstance() {
        if (mInstance == null) {
            mInstance = new RushinatorRollers();
        }
        return mInstance;
    }

    public void setFlywheelVoltage(double voltage) {
        mTalonWheel.setVoltage(voltage);
    }

    public void setFlywheelVelocity(Rotation2d velocity) {
        mTalonWheel.setControl(new VelocityVoltage(velocity.getRotations()));
    }


    public Rotation2d getWheelVelocity() {
        return Rotation2d.fromRotations(mTalonWheel.getVelocity().getValueAsDouble());
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Wheel Velocity (RPM)", getWheelVelocity().getRotations() * 60.0);
        
    }

    public static class DefaultCommand extends Command {
        RushinatorRollers subsystem;
        public DefaultCommand() {
            this.subsystem = RushinatorRollers.getInstance();
            addRequirements(subsystem);
        }

        @Override
        public void initialize() {
            subsystem.setFlywheelVoltage(0.4);
        }
    }
}
