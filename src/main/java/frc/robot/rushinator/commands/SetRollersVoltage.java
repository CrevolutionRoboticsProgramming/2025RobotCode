package frc.robot.rushinator.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.rushinator.RushinatorPivot;
import frc.robot.rushinator.RushinatorRollers;

public class SetRollersVoltage extends Command{
    RushinatorRollers mRollers;
    double reqVolts;
    public SetRollersVoltage(double voltage) {
        mRollers = RushinatorRollers.getInstance();
        reqVolts = voltage;
        
        addRequirements(mRollers);
    }

    @Override
    public void initialize() {
        mRollers.setFlywheelVoltage(reqVolts);
    }

    @Override
    public void end(boolean interrupted) {

    }
}
