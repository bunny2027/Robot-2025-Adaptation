// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.PreferenceTypes.DoublePreference;

@Logged
public class Climber implements Subsystem {
    private final TalonFX m_climberMotorLeader;

    private final Servo m_rightServo;
    private final Servo m_leftServo;

  private Slot0Configs m_Slot0Configs = new Slot0Configs();
  
  private SoftwareLimitSwitchConfigs m_softLimitConfig = new SoftwareLimitSwitchConfigs();

  private MotionMagicConfigs m_motionMagicConfigs = new MotionMagicConfigs();

  private MotorOutputConfigs m_leaderMotorConfig = new MotorOutputConfigs();

  private MotorOutputConfigs m_followerMotorConfig = new MotorOutputConfigs();

  private TalonFXConfiguration m_fxCfg = new TalonFXConfiguration();

  public MotionMagicVoltage m_MMPosition =   new MotionMagicVoltage(0);

  //logged stuff
  @Logged(name = "Target Position", importance = Importance.CRITICAL)
  private double m_targetPosition;

  @Logged(name = "kP", importance = Importance.CRITICAL)
  private double climberkP;

  @Logged(name = "kD", importance = Importance.CRITICAL)
  private double climberkD;

  @Logged(name = "kI", importance = Importance.CRITICAL)
  private double climberkI;

  @Logged(name = "kG", importance = Importance.CRITICAL)
  private double climberkG;

  @Logged(name = "Motion Magic Cruise Velocity", importance = Importance.CRITICAL)
  private double m_MMCruiseVelocity;

  @Logged(name = "Motion Magic Acceleration", importance = Importance.CRITICAL)
  private double m_MMAccel;

  @Logged(name = "Motion Magic Jerk", importance = Importance.CRITICAL)
  private double m_MMJerk;


  /** Creates a new Climber. */
  public Climber() {
    m_climberMotorLeader = ClimberConstants.CLIMBER_LEADER_MOTOR; 

    m_Slot0Configs = ClimberConstants.createSlot0Configs(); 
    m_climberMotorLeader.getConfigurator().apply(m_Slot0Configs);

    climberkP = m_Slot0Configs.kP;
    climberkD = m_Slot0Configs.kD;
    climberkI = m_Slot0Configs.kI;
    climberkG = m_Slot0Configs.kG;

    m_softLimitConfig = ClimberConstants.createSoftLimitConigs(); 
    m_climberMotorLeader.getConfigurator().apply(m_softLimitConfig);

    m_motionMagicConfigs = ClimberConstants.createMotionMagicConfigs();
    m_climberMotorLeader.getConfigurator().apply(m_motionMagicConfigs);

    m_MMCruiseVelocity = m_motionMagicConfigs.MotionMagicCruiseVelocity;
    m_MMAccel = m_motionMagicConfigs.MotionMagicAcceleration;
    m_MMJerk = m_motionMagicConfigs.MotionMagicJerk;

    m_leaderMotorConfig = ClimberConstants.createLeaderMotorOutputConfigs();
    m_climberMotorLeader.getConfigurator().apply(m_leaderMotorConfig);

    m_rightServo = new Servo(ClimberConstants.RIGHT_SERVO_PORT);
    m_leftServo = new Servo(ClimberConstants.LEFT_SERVO_PORT);

    m_climberMotorLeader.setPosition(0);


  }

  @NotLogged
  public void setPositionRevolutions(double position) {
    m_targetPosition = position;
    m_climberMotorLeader.setControl(m_MMPosition.withPosition(position).withSlot(0));
  }
  
  @NotLogged
  public void setPositionRevolutions(DoublePreference position){
    setPositionRevolutions(position.get());
  }

  @Logged
  public double getPosition(){
    return m_climberMotorLeader.getPosition().getValueAsDouble();
  }

  @NotLogged
  public void setSpeed(double speed){
    m_climberMotorLeader.set(speed);
  }

  @NotLogged
  public void setSpeed(DoublePreference speed){
    m_climberMotorLeader.set(speed.getValue());
  }

  public void setServoPosition(double position){
    m_rightServo.set(position);
    m_leftServo.set(1.0 - position);
  }

  public void setServoPosition(DoublePreference position){
    setServoPosition(position.get());
  }
  
  public void resetServoPosition(){
    m_rightServo.set(0);
    m_leftServo.set(1);
  }

  @Override
  public void simulationPeriodic() {

  }

  public void setClimberPID(DoublePreference P, DoublePreference D, DoublePreference I, DoublePreference G){
    m_Slot0Configs = new Slot0Configs();

    m_climberMotorLeader.getConfigurator().refresh(m_Slot0Configs);

    m_Slot0Configs.withKP(P.getValue()).withKD(D.getValue()).withKI(I.getValue()).withKG(G.getValue());

    m_climberMotorLeader.getConfigurator().apply(m_Slot0Configs);

    climberkP = m_Slot0Configs.kP;
    climberkD = m_Slot0Configs.kD;
    climberkI = m_Slot0Configs.kI;
    climberkG = m_Slot0Configs.kG;
  }

 public void setClimberMotionMagic(DoublePreference CV, DoublePreference A, DoublePreference J){
    m_motionMagicConfigs = new MotionMagicConfigs();
    m_climberMotorLeader.getConfigurator().refresh(m_motionMagicConfigs);

    m_motionMagicConfigs.withMotionMagicCruiseVelocity(CV.getValue()).withMotionMagicAcceleration(A.getValue()).withMotionMagicJerk(J.getValue());

    m_climberMotorLeader.getConfigurator().apply(m_motionMagicConfigs);

    m_MMCruiseVelocity = m_motionMagicConfigs.MotionMagicCruiseVelocity;
    m_MMAccel = m_motionMagicConfigs.MotionMagicAcceleration;
    m_MMJerk = m_motionMagicConfigs.MotionMagicJerk;
  }

   //NotLogged
  // public double getClimberkP(){
  //   m_Slot0Configs = new Slot0Configs();
  //   m_climberMotorLeader.getConfigurator().refresh(m_Slot0Configs);
  //   return m_Slot0Configs.kP;
  // }

  // @NotLogged
  // public double getClimberkD(){
  //   m_Slot0Configs = new Slot0Configs();
  //   m_climberMotorLeader.getConfigurator().refresh(m_Slot0Configs);
  //   return m_Slot0Configs.kD;
  // }

  // @NotLogged
  // public double getClimberkI(){
  //   m_Slot0Configs = new Slot0Configs();
  //   m_climberMotorLeader.getConfigurator().refresh(m_Slot0Configs);
  //   return m_Slot0Configs.kI;
  // }

  // @NotLogged
  // public double getClimberkG(){
  //   m_Slot0Configs = new Slot0Configs();
  //   m_climberMotorLeader.getConfigurator().refresh(m_Slot0Configs);
  //   return m_Slot0Configs.kG;
  // }

  @Logged(name = "Right Servo Position", importance = Importance.CRITICAL )
  public double getRightServoPosition(){
    return m_rightServo.getPosition();
  }

  @Logged(name = "Left Servo Position", importance = Importance.CRITICAL)
  public double getLeftServoPosition(){
    return m_leftServo.getPosition();
  }
  
  @Override
  public void periodic(){
    SmartDashboard.putNumber("Right Servo Position", m_rightServo.getPosition());
    SmartDashboard.putNumber("Left Servo Position", m_leftServo.getPosition());
  }

  //Voltage, Current, Temperature

  /*********Logging Motors*************/
  @Logged(name = "Leader Motor Voltage", importance = Importance.CRITICAL)
  public double getLeaderVoltage(){
    return m_climberMotorLeader.getMotorVoltage().getValueAsDouble();
  }

  @Logged(name = "Leader Motor Current", importance = Importance.CRITICAL)
  public double getLeaderCurrent(){
    return m_climberMotorLeader.getSupplyCurrent().getValueAsDouble();
  }

  @Logged(name = "Leader Motor Temperature", importance = Importance.CRITICAL)
  public double getLeaderTemperature(){
    return m_climberMotorLeader.getDeviceTemp().getValueAsDouble();
  }

}