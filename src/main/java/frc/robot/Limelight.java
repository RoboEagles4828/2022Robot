package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;

public class Limelight {

    //main network table
    public NetworkTable table;
    private NetworkTableEntry tx;
    private NetworkTableEntry ty;
    private NetworkTableEntry ta;
    private NetworkTableEntry tv;

    //distance stuff
    private double mountAngle;

    public Limelight(NetworkTable table) {
        this.table=table;
        tx = table.getEntry("tx");
        ty = table.getEntry("ty");
        ta = table.getEntry("ta");
        tv = table.getEntry("tv");
    }

    public Limelight(NetworkTable table, double mountAngle) {
        this.table = table;
        tx = table.getEntry("tx");
        ty = table.getEntry("ty");
        ta = table.getEntry("ta");
        tv = table.getEntry("tv");
        this.mountAngle = mountAngle;
    }

    public double getX() {
        return tx.getDouble(0);
    }

    public double getY() {
        return ty.getDouble(0);
    }

    public double getAngle() {
        return ta.getDouble(0);
    }

    public boolean hasValidTarget() {
        return tv.getDouble(0) == 1 ? true : false;
    }

    public void setLEDState(int state) {
        table.getEntry("ledMode").setNumber(state);
    }

    public double getDistanceFromTarget(double goalHeight, double camHeight) {
        double distance = 0;
        if (!hasValidTarget()) {
            distance = -1;
        }
        else {
            distance = (goalHeight - camHeight) / Math.tan((mountAngle + ty.getDouble(0)) * (Math.PI / 180));
        }
        return distance;
    }

    //-right +left
    public double getAlignmentAdjustment() {
        double min = 0.05; //tune
        double adjustment = 0;
        double kP = 0.01; //tune
        double headingErr = tx.getDouble(0);
        if (hasValidTarget()) {
            if (tx.getDouble(0) > 1) {
                adjustment = kP * headingErr - min;
            }
            else if (tx.getDouble(0) < 1) {
                adjustment = kP * headingErr + min;
            }
        }
        return adjustment;
    }
}