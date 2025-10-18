package frc.robot.subsystems.Elevator;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.PreferenceTypes.DoublePreference;
import frc.robot.statemachines.CoralState;

@Logged
public class Elevator implements Subsystem {
    private final TalonFX m_elevatorMotorLeader;
    private final TalonFX m_elevatorMotorFollower;

  private Slot0Configs m_Slot0Configs = new Slot0Configs();
  private Slot1Configs m_Slot1Configs = new Slot1Configs();
  
  private SoftwareLimitSwitchConfigs m_softLimitConfig = new SoftwareLimitSwitchConfigs();

  private MotionMagicConfigs m_motionMagicConfigs = new MotionMagicConfigs();

  private MotorOutputConfigs m_leaderMotorConfig = new MotorOutputConfigs();

  private MotorOutputConfigs m_followerMotorConfig = new MotorOutputConfigs();

  private TalonFXConfiguration m_fxCfg = new TalonFXConfiguration();

  public MotionMagicVoltage m_MMPosition =   new MotionMagicVoltage(0);

  public PositionVoltage m_PositionVoltage = new PositionVoltage(0);

  //logged stuff
  @Logged(name = "Target Position", importance = Importance.CRITICAL)
  private double m_targetPosition;

  @Logged(name = "kP", importance = Importance.CRITICAL)
  private double elevatorkP;

  @Logged(name = "kD", importance = Importance.CRITICAL)
  private double elevatorkD;

  @Logged(name = "kI", importance = Importance.CRITICAL)
  private double elevatorkI;

  @Logged(name = "kG", importance = Importance.CRITICAL)
  private double elevatorkG;

  @Logged(name = "kS", importance = Importance.CRITICAL)
  private double elevatorkS;

  @Logged(name = "Motion Magic Cruise Velocity", importance = Importance.CRITICAL)
  private double m_MMCruiseVelocity;

  @Logged(name = "Motion Magic Acceleration", importance = Importance.CRITICAL)
  private double m_MMAccel;

  @Logged(name = "Motion Magic Jerk", importance = Importance.CRITICAL)
  private double m_MMJerk;


  /** Creates a new Elevator. */
  public Elevator() {
    m_elevatorMotorLeader = ElevatorConstants.ELEVATOR_LEADER_MOTOR; 
    m_elevatorMotorFollower = ElevatorConstants.ELEVATOR_FOLLOWER_MOTOR;

    m_Slot0Configs = ElevatorConstants.createSlot0Configs(); 
    m_elevatorMotorLeader.getConfigurator().apply(m_Slot0Configs);
    m_elevatorMotorFollower.getConfigurator().apply(m_Slot0Configs);

    m_Slot1Configs = ElevatorConstants.createSlot1Configs();
    m_elevatorMotorLeader.getConfigurator().apply(m_Slot1Configs);
    m_elevatorMotorFollower.getConfigurator().apply(m_Slot1Configs);

    elevatorkP = m_Slot0Configs.kP;
    elevatorkD = m_Slot0Configs.kD;
    elevatorkI = m_Slot0Configs.kI;
    elevatorkG = m_Slot0Configs.kG;
    elevatorkS = m_Slot0Configs.kS;

    m_softLimitConfig = ElevatorConstants.createSoftLimitConigs(); 
    m_elevatorMotorLeader.getConfigurator().apply(m_softLimitConfig);
    m_elevatorMotorFollower.getConfigurator().apply(m_softLimitConfig);

    m_motionMagicConfigs = ElevatorConstants.createMotionMagicConfigs();
    m_elevatorMotorLeader.getConfigurator().apply(m_motionMagicConfigs);
    m_elevatorMotorFollower.getConfigurator().apply(m_motionMagicConfigs);

    m_MMCruiseVelocity = m_motionMagicConfigs.MotionMagicCruiseVelocity;
    m_MMAccel = m_motionMagicConfigs.MotionMagicAcceleration;
    m_MMJerk = m_motionMagicConfigs.MotionMagicJerk;

    m_leaderMotorConfig = ElevatorConstants.createLeaderMotorOutputConfigs();
    m_elevatorMotorLeader.getConfigurator().apply(m_leaderMotorConfig);

    m_followerMotorConfig = ElevatorConstants.createFollowerMotorOutputConfigs();
    m_elevatorMotorFollower.getConfigurator().apply(m_followerMotorConfig);

    reset();

  }

  @NotLogged
  public void setPositionRevolutions(double position) {
    if(!CoralState.getInstance().isBlocked()){
      m_targetPosition = position;
      //m_elevatorMotorLeader.setControl(m_MMPosition.withPosition(position).withSlot(0));
      m_elevatorMotorLeader.setControl(m_MMPosition.withPosition(position).withSlot(0));
    }
  }
  
  @NotLogged
  public void setPositionRevolutions(DoublePreference position){
    setPositionRevolutions(position.get());
  }


  public void setSlowPositionRevolutions(double position) {
    if(!CoralState.getInstance().isBlocked()){
      m_targetPosition = position;
      //m_elevatorMotorLeader.setControl(m_MMPosition.withPosition(position).withSlot(0));
      m_elevatorMotorLeader.setControl(
        m_PositionVoltage
          .withPosition(position)
          .withVelocity(ElevatorConstants.ELEVATOR_SLOW_VELOCITY)
          .withSlot(1)
      );
    }
  }
  
  @NotLogged
  public void setSlowPositionRevolutions(DoublePreference position){
    setSlowPositionRevolutions(position.get());
  }

  @Logged
  public double getPosition(){
    return m_elevatorMotorLeader.getPosition().getValueAsDouble();
  }

  @Logged
  public double getTargetPosition(){
    return m_targetPosition;
  
  }
  
  @Logged
  public boolean atSetpoint(){
    return((getPosition() - ElevatorConstants.POSITION_ERROR < m_targetPosition) && (getPosition()+ElevatorConstants.POSITION_ERROR > m_targetPosition));
  }

  public void reset(){
    m_elevatorMotorLeader.setPosition(0);
    m_elevatorMotorFollower.setPosition(0);
  }

  public void stopMotors(){
    m_elevatorMotorLeader.stopMotor();
  }


  @Override
  public void periodic() {
    
  }

  @Override
  public void simulationPeriodic() {

  }

  public void setElevatorPID(DoublePreference P, DoublePreference D, DoublePreference I, DoublePreference G, DoublePreference S){
    m_Slot0Configs = new Slot0Configs();

    m_elevatorMotorLeader.getConfigurator().refresh(m_Slot0Configs);

    m_Slot0Configs.withKP(P.getValue()).withKD(D.getValue()).withKI(I.getValue()).withKG(G.getValue()).withKS(S.getValue());

    m_elevatorMotorLeader.getConfigurator().apply(m_Slot0Configs);
    m_elevatorMotorFollower.getConfigurator().apply(m_Slot0Configs);

    elevatorkP = m_Slot0Configs.kP;
    elevatorkD = m_Slot0Configs.kD;
    elevatorkI = m_Slot0Configs.kI;
    elevatorkG = m_Slot0Configs.kG;
    elevatorkS = m_Slot0Configs.kS;
  }

  public void setElevatorMotionMagic(DoublePreference CV, DoublePreference A, DoublePreference J){
    m_motionMagicConfigs = new MotionMagicConfigs();
    m_elevatorMotorLeader.getConfigurator().refresh(m_motionMagicConfigs);

    m_motionMagicConfigs.withMotionMagicCruiseVelocity(CV.getValue()).withMotionMagicAcceleration(A.getValue()).withMotionMagicJerk(J.getValue());

    m_elevatorMotorLeader.getConfigurator().apply(m_motionMagicConfigs);
    m_elevatorMotorFollower.getConfigurator().apply(m_motionMagicConfigs);

    m_MMCruiseVelocity = m_motionMagicConfigs.MotionMagicCruiseVelocity;
    m_MMAccel = m_motionMagicConfigs.MotionMagicAcceleration;
    m_MMJerk = m_motionMagicConfigs.MotionMagicJerk;
  }

  // @NotLogged
  // public double getElevatorkP(){
  //   m_Slot0Configs = new Slot0Configs();
  //   m_elevatorMotorLeader.getConfigurator().refresh(m_Slot0Configs);
  //   return m_Slot0Configs.kP;
  // }

  // @NotLogged
  // public double getElevatorkD(){
  //   m_Slot0Configs = new Slot0Configs();
  //   m_elevatorMotorLeader.getConfigurator().refresh(m_Slot0Configs);
  //   return m_Slot0Configs.kD;
  // }

  // @NotLogged
  // public double getElevatorkI(){
  //   m_Slot0Configs = new Slot0Configs();
  //   m_elevatorMotorLeader.getConfigurator().refresh(m_Slot0Configs);
  //   return m_Slot0Configs.kI;
  // }

  // @NotLogged
  // public double getElevatorkG(){
  //   m_Slot0Configs = new Slot0Configs();
  //   m_elevatorMotorLeader.getConfigurator().refresh(m_Slot0Configs);
  //   return m_Slot0Configs.kG;
  // }

  // @NotLogged
  // public double getElevatorkS(){
  //   m_Slot0Configs = new Slot0Configs();
  //   m_elevatorMotorLeader.getConfigurator().refresh(m_Slot0Configs);
  //   return m_Slot0Configs.kS;
  // }

  // @NotLogged
  // public double getElevatorMMAccel(){
  //   m_motionMagicConfigs = new MotionMagicConfigs();
  //   m_elevatorMotorLeader.getConfigurator().refresh(m_motionMagicConfigs);
  //   return m_motionMagicConfigs.MotionMagicAcceleration;
  // }

  // @NotLogged
  // public double getElevatorMMJerk(){
  //   m_motionMagicConfigs = new MotionMagicConfigs();
  //   m_elevatorMotorLeader.getConfigurator().refresh(m_motionMagicConfigs);
  //   return m_motionMagicConfigs.MotionMagicJerk;
  // }

  // @NotLogged
  // public double getElevatorMMCruiseVelocity(){
  //   m_motionMagicConfigs = new MotionMagicConfigs();
  //   m_elevatorMotorLeader.getConfigurator().refresh(m_motionMagicConfigs);
  //   return m_motionMagicConfigs.MotionMagicCruiseVelocity;
  // }


  //Voltage, Current, Temperature

  /*********Logging Motors*************/
  @Logged(name = "Leader Motor Voltage", importance = Importance.CRITICAL)
  public double getLeaderVoltage(){
    return m_elevatorMotorLeader.getMotorVoltage().getValueAsDouble();
  }

  @Logged(name = "Leader Motor Current", importance = Importance.CRITICAL)
  public double getLeaderCurrent(){
    return m_elevatorMotorLeader.getSupplyCurrent().getValueAsDouble();
  }

  @Logged(name = "Leader Motor Temperature", importance = Importance.CRITICAL)
  public double getLeaderTemperature(){
    return m_elevatorMotorLeader.getDeviceTemp().getValueAsDouble();
  }

  @Logged(name = "Follower Motor Voltage", importance = Importance.CRITICAL)
  public double getFollowerVoltage(){
    return m_elevatorMotorFollower.getMotorVoltage().getValueAsDouble();
  }

  @Logged(name = "Follower Motor Current", importance = Importance.CRITICAL)
  public double getFollowerCurrent(){
    return m_elevatorMotorFollower.getSupplyCurrent().getValueAsDouble();
  }

  @Logged(name = "Follower Motor Temperature", importance = Importance.CRITICAL)
  public double getFollowerTemperature(){
    return m_elevatorMotorFollower.getDeviceTemp().getValueAsDouble();
  }



}
