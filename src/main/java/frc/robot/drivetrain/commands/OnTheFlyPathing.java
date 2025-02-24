package frc.robot.drivetrain.commands;

import java.util.Optional;
import java.util.function.Supplier;

import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.aprilTagPoses.CoralGoalPose2d;
import frc.robot.aprilTagPoses.CoralGoalPose2d.*;
import frc.robot.coralator.Coralator;
import frc.robot.drivetrain.Drivetrain;
import frc.robot.vision.Vision;

public class OnTheFlyPathing extends Command{
    Vision[] mVision;
    Drivetrain mDrivetrain;
    CoralGoalPose2d mGoalPose2d;
    int tagNum;
    boolean right;

    OnTheFlyPathing(int tagNumber, boolean right) {
        mVision = Vision.getInstance();
        mDrivetrain = Drivetrain.getInstance();
        mGoalPose2d = CoralGoalPose2d.getInstance();
        tagNum = tagNumber;
        this.right = right;
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        Pose2d goalPose = null;
        if (right){
            
        }else {

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
