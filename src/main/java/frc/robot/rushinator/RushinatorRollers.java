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
        static final int kTopID = 15;
        static final int kBottomID = 16;

        // RPS
        static final Rotation2d kMaxAngluarVelocity = Rotation2d.fromRotations(6000 / 60);
    }

    private static RushinatorRollers mInstance;
    private final TalonFX mTalonTop, mTalonBottom;

    private RushinatorRollers() {
        mTalonTop = new TalonFX(Settings.kTopID);
        mTalonBottom = new TalonFX(Settings.kBottomID);
    }

    public static RushinatorRollers getInstance() {
        if (mInstance == null) {
            mInstance = new RushinatorRollers();
        }
        return mInstance;
    }

    public void setFlywheelVoltage(double voltage) {
        mTalonTop.setVoltage(voltage);
        mTalonBottom.setVoltage(voltage);
    }

    public void setFlywheelVelocity(Rotation2d velocity) {
        mTalonTop.setControl(new VelocityVoltage(velocity.getRotations()));
        mTalonBottom.setControl(new VelocityVoltage(velocity.getRotations()));
    }


    public Rotation2d getTopWheelVelocity() {
        return Rotation2d.fromRotations(mTalonTop.getVelocity().getValueAsDouble());
    }

    public Rotation2d getBottomwWheelVelocity() {
        return Rotation2d.fromRotations(mTalonBottom.getVelocity().getValueAsDouble());
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Top Wheel Velocity (RPM)", getTopWheelVelocity().getRotations() * 60.0);
        SmartDashboard.putNumber("Bottom Wheel Velocity (RPM)", getBottomwWheelVelocity().getRotations() * 60.0);
    }

    public static class DefaultCommand extends Command {
        RushinatorRollers subsystem;
        public DefaultCommand() {
            this.subsystem = RushinatorRollers.getInstance();
            addRequirements(subsystem);
        }

        @Override
        public void initialize() {
            subsystem.setFlywheelVoltage(0);
        }
    }
}
