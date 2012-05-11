package testsystem;

import java.util.Vector;


//IMU Entry
public class IMUEntry {
	Vector <Double> accel = new Vector<Double>();
	Vector <Double> raccel = new Vector<Double>();
	Double timestamp;
	
	public IMUEntry(Vector<Double> d_accel, Vector<Double> d_raccel, Double timestamp) {
		this.accel = d_accel;
		this.raccel = d_raccel;
		this.timestamp = timestamp;
	}
	
	public void setIMUAcceleration(Vector<Double> d_accel) {
		this.accel = d_accel;			
	}
	
	public Vector<Double> getIMUAcceleration() {
		return accel;
	}
	
	public void setIMURotAccel(Vector<Double> d_raccel) {
		this.raccel = d_raccel;
	}

	public Vector<Double> getIMURotAccel() {
		return raccel;
	}
	

	public void setIMUTimestamp(Double timestamp) {
		this.timestamp = timestamp;
	}

	public Double getIMUTimestamp() {
		return timestamp;
	}
	
	public IMUEntry copyIMUEntry(){
		IMUEntry retval = new IMUEntry(accel, raccel, timestamp);
		return retval;
	}

	public String toString() {
        String retval = "   Acceleration Vector:\n      [" + accel.get(0) + ", " + accel.get(1) + ", " + accel.get(2) + "]\n";
        retval = retval + "   Rotational Acceleration Vector:\n      [" + raccel.get(0) + ", " + raccel.get(1) + ", " + raccel.get(2) + "]\n";
        retval = retval + "   Timestamp = " + timestamp;
        return retval;
    }
	
}
