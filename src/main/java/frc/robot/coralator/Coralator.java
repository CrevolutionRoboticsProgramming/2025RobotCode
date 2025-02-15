package frc.robot.coralator;

import java.lang.module.Configuration;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Coralator extends SubsystemBase{
    public static class Settings {
        static final int kCoralatorId = 29;

        static final InvertedValue kCoralatorScoringInverted = InvertedValue.Clockwise_Positive;
        static final InvertedValue kCoralatorIntakingInverted = InvertedValue.CounterClockwise_Positive;

        static final double kDetectionThreshold = 20.0;

        static final Slot0Configs kFlywheelConfigs = new Slot0Configs()
                .withKS(0.0)
                .withKV(0.115)
                .withKA(0.1)
                .withKP(0.01)
        ;

        // 5800 RPM at motor; 11600 RPM at wheels
        public static final Rotation2d kMaxAngularVelocity = Rotation2d.fromRotations(6000.0 / 60.0);

        public static final double kCurrentLimit = 60.0;
    }
    public static Coralator mInstance;
    private TalonFXConfigurator CoralatorConfigurator;
    private MotorOutputConfigs CoralatorConfigs;

    private TalonFX Coralator;
    
    public Coralator() {
        Coralator = new TalonFX(Settings.kCoralatorId);
        CoralatorConfigurator = Coralator.getConfigurator();
        CoralatorConfigs = new MotorOutputConfigs();
        
        CoralatorConfigurator.apply(Settings.kFlywheelConfigs);
        
    }

    public static Coralator getInstance() {
        if (mInstance == null) {
            mInstance = new Coralator();
        }
        return mInstance;
    }

    public void setVelocity(Rotation2d velocity, InvertedValue kInvertedValue) {
        // set invert to CW+ and apply config change
        CoralatorConfigs.Inverted = kInvertedValue;
        CoralatorConfigurator.apply(CoralatorConfigs);

        Coralator.setControl(new VelocityVoltage(velocity.getRotations()));
    }

    public double getVelocity() {
        // returns in Rps
        return Coralator.getVelocity().getValueAsDouble();
    }

    public boolean hasCoral() {
        return Coralator.getStatorCurrent().getValueAsDouble() > Settings.kDetectionThreshold;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Coral Velocity (RPM)", Coralator.getVelocity().getValueAsDouble() * 60);
        
       
    }
}
