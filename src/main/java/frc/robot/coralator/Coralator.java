package frc.robot.coralator;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Coralator extends SubsystemBase{
    public static class Settings {
        static final int kCoralatorId = 29;

        static final InvertedValue kCoralatorInverted = InvertedValue.Clockwise_Positive;
        
    }
    public static Coralator mInstance;

    private TalonFX Coralator;


    public Coralator() {
        Coralator = new TalonFX(Settings.kCoralatorId);

        var CoralatorConfigurator = Coralator.getConfigurator();
        var CoralatorConfigs = new MotorOutputConfigs();
        // set invert to CW+ and apply config change
        CoralatorConfigs.Inverted = Settings.kCoralatorInverted;
        CoralatorConfigurator.apply(CoralatorConfigs);
    }

    public static Coralator getInstance() {
        if (mInstance == null) {
            mInstance = new Coralator();
        }
        return mInstance;
    }

    
}
