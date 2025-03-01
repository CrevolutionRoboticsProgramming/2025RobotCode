package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CoralRollerSubsystem extends SubsystemBase {
    private static CoralRollerSubsystem mInstance;
    private TalonFX mTalonRoller;

    private CoralRollerSubsystem() {
        mTalonRoller = new TalonFX(13);

    }

    public static CoralRollerSubsystem getInstance() {
        if (mInstance == null) {
            mInstance = new CoralRollerSubsystem();
        }
        return mInstance;
    }

    void setVoltage(double voltage) {
        mTalonRoller.setVoltage(voltage);
    }

    public static class SetVoltageCommand extends Command {
        double volts;
        public SetVoltageCommand(double volts) {
            this.volts = volts;
            addRequirements(CoralRollerSubsystem.getInstance());
        }

        @Override
        public void initialize() {
            CoralRollerSubsystem.getInstance().setVoltage(volts);
        }
    }
}
