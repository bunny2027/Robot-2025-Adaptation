package frc.robot.subsystems.coral;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.ProximityParamsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.Units;

public class CorralerConstants {
    public static final int kCoralMotorId = 2;

    public static final double CORAL_kV = 0;
    public static final double CORAL_kS = 0;
    public static final double CORAL_kP = 0;
    public static final double CORAL_kI = 0;
    public static final double CORAL_kD = 0;

    public static final double INTAKE_CORAL_VOLTAGE = 1;
    public static final double OUTTAKE_CORAL_VOLTAGE = -1;

    public static final double INTAKE_CORAL_POWER = 0.25;
    public static final double OUTTAKE_CORAL_POWER = 0.25;

    public static final int kEnterBeamBreakId = 8;
    public static final int kPrepBeamBreakId = 9;

    public static final double OUTTAKE_DELAY = 0.5;
    public static Slot0Configs createCoralMotorSlot0Configs(){
        Slot0Configs slot = new Slot0Configs();
        slot.kV = CorralerConstants.CORAL_kV;
        slot.kS = CorralerConstants.CORAL_kS;
        slot.kP = CorralerConstants.CORAL_kP;
        slot.kI = CorralerConstants.CORAL_kI;
        slot.kD = CorralerConstants.CORAL_kD;
        return slot;
    }

    public static MotorOutputConfigs createCoralMotorOutputConfigs(){
        MotorOutputConfigs configs = new MotorOutputConfigs();
        configs.withInverted(InvertedValue.Clockwise_Positive);
        configs.NeutralMode = NeutralModeValue.Brake;
        return configs;
    }
}
