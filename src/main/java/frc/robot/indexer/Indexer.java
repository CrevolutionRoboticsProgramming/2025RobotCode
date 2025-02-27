package frc.robot.indexer;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Indexer extends SubsystemBase{
  public static class Settings {
      static int kTalonID = 17;
      static final double kMaxVoltage = 12.0;
      static final int kCurrentLimit = 200;
      static final int kCurrentThreshold = 100;
  }

  private static Indexer mInstance;
  private TalonFX mKraken;
  
  public Indexer() {
    mKraken = new TalonFX(Settings.kTalonID);
    
    var talonFXConfigurator = mKraken.getConfigurator();
    var motorConfigs = new MotorOutputConfigs();

    motorConfigs.Inverted = InvertedValue.CounterClockwise_Positive;
    talonFXConfigurator.apply(motorConfigs);

  }

  public static Indexer getInstance() {
      if (mInstance == null) {
          mInstance = new Indexer();
      }
      return mInstance;
  }

  public void setOutput(double percentOut) {
      mKraken.setVoltage(percentOut * Settings.kMaxVoltage);
  }

  public Boolean hasAlgae() {
    //needs testing
    return (mKraken.getStatorCurrent().getValueAsDouble() > Settings.kCurrentThreshold);
  }

 @Override
 public void periodic() {
  SmartDashboard.putBoolean("Indexer Has Algae", hasAlgae());
 }
}
