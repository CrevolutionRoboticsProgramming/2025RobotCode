package frc.robot.coralator;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CoralRollerSubsystem extends SubsystemBase {
    private static CoralRollerSubsystem mInstance;
    private static TalonFX mTalonRoller;
    
    private CoralRollerSubsystem() {
        mTalonRoller = new TalonFX(13);

    }

    public static CoralRollerSubsystem getInstance() {
        if (mInstance == null) {
            mInstance = new CoralRollerSubsystem();
        }
        return mInstance;
    }

    public void setVoltage(double voltage) {
        mTalonRoller.setVoltage(voltage);
    }      
}
