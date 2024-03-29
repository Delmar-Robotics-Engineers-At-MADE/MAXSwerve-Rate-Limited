package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import edu.wpi.first.networktables.GenericEntry;
// import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
// import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class BlinkinSubsystem extends SubsystemBase{

  private final Spark m_blinkinPort = new Spark(0);
  private final Spark m_blinkinStar = new Spark(1);

  public double m_defaultSetting = 0.91; // violet
  public double m_signalCubes = 0.96; // ??
  public double m_signalCones = 0.77; // green
  public double m_signalWhite = 0.93;
  public double m_signalBlue = 0.87;
  public double m_signalYellow = 0.69;

  // private ShuffleboardTab m_dashboardTab;
  // private GenericEntry m_defaultSettingEntry;
  // private GenericEntry m_conesSettingEntry;
  // private GenericEntry m_cubesSettingEntry;

  public BlinkinSubsystem() {
    // m_dashboardTab = Shuffleboard.getTab("Lights");
    // m_defaultSettingEntry = m_dashboardTab.add("Default", m_defaultSetting).getEntry();
    // m_conesSettingEntry = m_dashboardTab.add("Cubes", m_signalCubes).getEntry();
    // m_cubesSettingEntry = m_dashboardTab.add("Cones", m_signalCones).getEntry();
    defaultLighting();  // start with default lighting
  }

  public void defaultLighting() {
    double pwmSetting = m_defaultSetting; //m_defaultSettingEntry.getDouble(m_defaultSetting);
    m_blinkinPort.set(pwmSetting);
    m_blinkinStar.set(pwmSetting);
    // System.out.println("setting default lighting: " + pwmSetting);
  }

  public void signalCubes() {
    double pwmSetting = m_signalCubes; // m_conesSettingEntry.getDouble(m_signalCubes);
    m_blinkinPort.set(pwmSetting);
    m_blinkinStar.set(pwmSetting);
    // System.out.println("signaling cubes: " + pwmSetting);
  }
  
  public void signalCones() {
    double pwmSetting = m_signalCones; // m_cubesSettingEntry.getDouble(m_signalCones);
    m_blinkinPort.set(pwmSetting);
    m_blinkinStar.set(pwmSetting);
    // System.out.println("signaling cones: " + pwmSetting);
  }

  public void signalA() {
    double pwmSetting = m_signalWhite; // m_cubesSettingEntry.getDouble(m_signalCones);
    m_blinkinPort.set(pwmSetting);
    m_blinkinStar.set(pwmSetting);
    // System.out.println("signaling cones: " + pwmSetting);
  }
  
  public void signalB() {
    double pwmSetting = m_signalBlue; // m_cubesSettingEntry.getDouble(m_signalCones);
    m_blinkinPort.set(pwmSetting);
    m_blinkinStar.set(pwmSetting);
    // System.out.println("signaling cones: " + pwmSetting);
  }
  
  public void signalC() {
    double pwmSetting = m_signalYellow; // m_cubesSettingEntry.getDouble(m_signalCones);
    m_blinkinPort.set(pwmSetting);
    m_blinkinStar.set(pwmSetting);
    // System.out.println("signaling cones: " + pwmSetting);
  }
  

  
}
