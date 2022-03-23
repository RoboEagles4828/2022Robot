package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import frc.robot.Constants.*;

public class Shooter {
    public WPI_TalonSRX shooter;

    public Shooter(){
        shooter = new WPI_TalonSRX(Ports.shooter_port);
    }

    public void set_speed(double speed){
        shooter.set(speed*-1);
    }

    public void set_voltage(double voltage){
        shooter.setVoltage(voltage);
    }

    public void stop(){
        shooter.set(0);
    }
}
