package frc.crevolib.util;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.util.Units;

public class WCPConstants {
    public final double wheelDiameter;
    public final double wheelCircumference;
    public final double angleGearRatio;
    public final double driveGearRatio;
    public final double angleKP;
    public final double angleKI;
    public final double angleKD;
    public final InvertedValue driveMotorInvert;
    public final InvertedValue angleMotorInvert;
    public final SensorDirectionValue cancoderInvert;
    

    public WCPConstants(double wheelDiameter, double angleGearRatio, double driveGearRatio, 
                            double angleKP, double angleKI, double angleKD, InvertedValue driveMotorInvert, 
                                InvertedValue angleMotorInvert, SensorDirectionValue cancoderInvert){
        this.wheelDiameter = wheelDiameter;
        this.wheelCircumference = wheelDiameter * Math.PI;
        this.angleGearRatio = angleGearRatio;
        this.driveGearRatio = driveGearRatio;
        this.angleKP = angleKP;
        this.angleKI = angleKI;
        this.angleKD = angleKD;
        this.driveMotorInvert = driveMotorInvert;
        this.angleMotorInvert = angleMotorInvert;
        this.cancoderInvert = cancoderInvert;
    }


    public static final class X2t  {
        public static final WCPConstants Falcon500(double driveGearRatio){
            double wheelDiameter = Units.inchesToMeters(4.0);
    
            double angleGearRatio = ( 12.1 / 1.0); // MAY NEE TO CHANGE
    
            double angleKP = 100.0;
            double angleKI = 0.0;
            double angleKD = 0.0;
    
            InvertedValue driveMotorInvert = InvertedValue.CounterClockwise_Positive;
            InvertedValue angleMotorInvert = InvertedValue.Clockwise_Positive;
            SensorDirectionValue cancoderInvert = SensorDirectionValue.CounterClockwise_Positive;
            return new WCPConstants(wheelDiameter, angleGearRatio, driveGearRatio, angleKP, angleKI, angleKD, driveMotorInvert, angleMotorInvert, cancoderInvert);
        }

        public static final class driveRatios{
            public static final double ratio = (5.15 / 1.0);
        }
    }

            
}
