// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.controls.Follower;

public class ClimberConstants {

        //Motor
        public static final int kclimberMotorLeaderId = 15;
        public static final TalonFX CLIMBER_LEADER_MOTOR = new TalonFX(kclimberMotorLeaderId);

        public static final int RIGHT_SERVO_PORT = 0;
        public static final int LEFT_SERVO_PORT = 1;


        //Slot0Configs

        public static final double CLIMBER_kV = 0;
        public static final double CLIMBER_kS = 0;
        public static final double CLIMBER_kP = 5;
        public static final double CLIMBER_kI = 0;
        public static final double CLIMBER_kD = 0;
        public static final double CLIMBER_kG = 0;
        public static final GravityTypeValue CLIMBER_GRAVITY = GravityTypeValue.Elevator_Static;

        public static Slot0Configs createSlot0Configs(){ 
            Slot0Configs slot = new Slot0Configs();
            slot.kV = CLIMBER_kV;
            slot.kS = CLIMBER_kS;
            slot.kP = CLIMBER_kP;
            slot.kI = CLIMBER_kI;
            slot.kD = CLIMBER_kD;
            slot.kG = CLIMBER_kG;
            slot.GravityType = CLIMBER_GRAVITY;
           return slot; 
        }

        //SoftLimitConfig
        public static final double CLIMBER_FORWARD_SOFT_LIMIT = 100;
        public static final double CLIMBER_REVERSE_SOFT_LIMIT = 0.04;

        public static SoftwareLimitSwitchConfigs createSoftLimitConigs(){
            SoftwareLimitSwitchConfigs newConfigs = new SoftwareLimitSwitchConfigs();
            newConfigs.ForwardSoftLimitEnable = false;
            newConfigs.ReverseSoftLimitEnable = false;
            newConfigs.ForwardSoftLimitThreshold = CLIMBER_FORWARD_SOFT_LIMIT;
             newConfigs.ReverseSoftLimitThreshold = CLIMBER_REVERSE_SOFT_LIMIT;
                return newConfigs;
        }

        //MotionMagicConfigs
        public static final double CLIMBER_MM_JERK = 1000;
        public static final double CLIMBER_MM_ACCEL = 100;
        public static final double CLIMBER_MM_CRUISE_VELOCITY = 50;
        public static MotionMagicConfigs createMotionMagicConfigs(){
            MotionMagicConfigs newConfigs = new MotionMagicConfigs();
            newConfigs.MotionMagicJerk = CLIMBER_MM_JERK;
            newConfigs.MotionMagicAcceleration = CLIMBER_MM_ACCEL;
            newConfigs.MotionMagicCruiseVelocity = CLIMBER_MM_CRUISE_VELOCITY;
            return newConfigs;
        }

        public static MotorOutputConfigs createLeaderMotorOutputConfigs(){
            MotorOutputConfigs newConfigs = new MotorOutputConfigs();
            newConfigs.Inverted = InvertedValue.CounterClockwise_Positive;
            newConfigs.NeutralMode = NeutralModeValue.Brake;
            return newConfigs;
        }


}