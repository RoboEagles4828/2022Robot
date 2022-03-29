package frc.robot;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

public class Shooter {
    public WPI_TalonSRX shooter;

    public Shooter(int port){
        shooter = new WPI_TalonSRX(port);
        shooter.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
    }

    public void set_speed(double speed){
        shooter.set(speed*-1);
    }

    public void set_voltage(double voltage){
        shooter.setVoltage(voltage);
    }

    public double get_velocity(){
        return shooter.getSelectedSensorVelocity();
    }

    public double get_pos(){
        return shooter.getSelectedSensorPosition();
    }

    public boolean is_ready(double vel){
        if(get_velocity()>vel){
            return true;
        }
        return false;
    }
    public void stop(){
        shooter.set(0);
    }
}
