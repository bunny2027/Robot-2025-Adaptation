// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.algae;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

/** Add your docs here. */
public class AlgaeCollectorConstants {

    public static final double WRIST_POSITION_ERROR = 0.1;

    public static final int kAlgaeMotorId = 3;
    public static final int kWristMotorId = 4;

    public static final int kAlgaeBeamBreakId = 16;

    public static final double ALGAE_kV = 0;
    public static final double ALGAE_kS = 0;
    public static final double ALGAE_kP = 0;
    public static final double ALGAE_kI = 0;
    public static final double ALGAE_kD = 0;

    public static final double WRIST_kV_0 = 0;
    public static final double WRIST_kS_0 = 0.3;
    public static final double WRIST_kP_0 = 20;
    public static final double WRIST_kI_0 = 0;
    public static final double WRIST_kD_0 = 0;
    public static final double WRIST_kG_0 = 0.14;

    public static final double WRIST_kV_1 = 0;
    public static final double WRIST_kS_1 = 0;
    public static final double WRIST_kP_1 = 0;
    public static final double WRIST_kI_1 = 0;
    public static final double WRIST_kD_1 = 0;
    public static final double WRIST_kG_1 = 0.2;

    public static final double INTAKE_CORAL_VOLTAGE = 1;
    public static final double OUTTAKE_CORAL_VOLTAGE = -1;

    public static final double INTAKE_CORAL_POWER = 0.25;
    public static final double OUTTAKE_CORAL_POWER = 0.25;

    public static final double INTAKE_ALGAE_POWER = 0.25;
    public static final double OUTTAKE_ALGAE_POWER = 0.25;
    public static final double ALGAE_HOLD_POWER = 0.03;

    public static final double ALGAE_INTAKE_CURRENT_LIMIT = 20;


    public static MotorOutputConfigs createAlgaeMotorOutputConfigs(){
        MotorOutputConfigs configs = new MotorOutputConfigs();
        configs.withInverted(InvertedValue.Clockwise_Positive);
        configs.NeutralMode = NeutralModeValue.Brake;
        return configs;
    }

    public static Slot0Configs createAlgaeMotorSlot0Configs(){
        Slot0Configs slot = new Slot0Configs();
        slot.kV = AlgaeCollectorConstants.ALGAE_kV;
        slot.kS = AlgaeCollectorConstants.ALGAE_kS;
        slot.kP = AlgaeCollectorConstants.ALGAE_kP;
        slot.kI = AlgaeCollectorConstants.ALGAE_kI;
        slot.kD = AlgaeCollectorConstants.ALGAE_kD;
        return slot;
    }

    public static CurrentLimitsConfigs createCurrentLimitsConfigs(){
        CurrentLimitsConfigs configs = new CurrentLimitsConfigs();
        // configs.StatorCurrentLimit = 25;
        // configs.StatorCurrentLimitEnable = true;
        return configs;
    }

    public static final StaticFeedforwardSignValue wristFeedforward = StaticFeedforwardSignValue.UseClosedLoopSign;
    public static final GravityTypeValue wristGravityType = GravityTypeValue.Arm_Cosine;
    public static Slot0Configs createWristMotorSlot0Configs(){
        Slot0Configs slot = new Slot0Configs();
        slot.kV = AlgaeCollectorConstants.WRIST_kV_0;
        slot.kS = AlgaeCollectorConstants.WRIST_kS_0;
        slot.kP = AlgaeCollectorConstants.WRIST_kP_0;
        slot.kI = AlgaeCollectorConstants.WRIST_kI_0;
        slot.kD = AlgaeCollectorConstants.WRIST_kD_0;
        slot.kG = AlgaeCollectorConstants.WRIST_kG_0;
        slot.StaticFeedforwardSign = wristFeedforward;
        slot.GravityType = wristGravityType;
        return slot;
    }

    public static Slot1Configs createWristMotorSlot1Configs(){
        Slot1Configs slot = new Slot1Configs();
        slot.kV = AlgaeCollectorConstants.WRIST_kV_1;
        slot.kS = AlgaeCollectorConstants.WRIST_kS_1;
        slot.kP = AlgaeCollectorConstants.WRIST_kP_1;
        slot.kI = AlgaeCollectorConstants.WRIST_kI_1;
        slot.kD = AlgaeCollectorConstants.WRIST_kD_1;
        slot.kG = AlgaeCollectorConstants.WRIST_kG_1;
        slot.StaticFeedforwardSign = wristFeedforward;
        slot.GravityType = wristGravityType;
        return slot;
    }

    public static final double WRIST_FORWARD_SOFT_LIMIT = 100;
    public static final double WRIST_REVERSE_SOFT_LIMIT = 0;
    public static final int WRIST_TOLERANCE = 0;

    public static SoftwareLimitSwitchConfigs createWristSoftLimitConfigs(){
        SoftwareLimitSwitchConfigs configs = new SoftwareLimitSwitchConfigs();
        configs.ForwardSoftLimitEnable = false;
        configs.ReverseSoftLimitEnable = false;
        configs.ForwardSoftLimitThreshold = WRIST_FORWARD_SOFT_LIMIT;
        configs.ReverseSoftLimitThreshold = WRIST_REVERSE_SOFT_LIMIT;
        return configs;
    }
    
    public static TalonFXConfiguration createWristTalonFXConfigs(){
        TalonFXConfiguration configs = new TalonFXConfiguration();
        configs.Feedback.SensorToMechanismRatio = 48;
        return configs;
    }

    public static MotorOutputConfigs createWristMotorOutputConfigs(){
        MotorOutputConfigs configs = new MotorOutputConfigs();
        configs.Inverted = InvertedValue.Clockwise_Positive;
        return configs;
    }



    public enum WRIST{
        
        PROCESS(0.12),
        REEF(0.092),
        BARGE(0.23),
        STOWFULL(0.23),
        STOWEMPTY(0.23);

        public final double angle;
        WRIST(double value){
            angle = value;
        }

    }
}
