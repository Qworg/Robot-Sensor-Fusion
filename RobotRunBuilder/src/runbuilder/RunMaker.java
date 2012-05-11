package runbuilder;

import java.util.*;

public class RunMaker {
	private boolean GPSMal;

	private boolean IMUMal;

	private boolean CompassMal;

	private boolean OdoMal;

	private boolean GPSBrid;

	private Integer RunLength;

	private Integer CurrentRun;

	private boolean StepLock, OutputLock, DLock, CLock;

	private boolean Finished, Updated;

	// private static Double TimestepSize = 0.0056;
	private String sDataSpace = "	";

	private static Double TimestepSize = 1.0; // In ms //0.0001;

	private static int MalfunProb = 100000; // As in 1/Xnd

	private static Double GPSErr = 2.5;

	private static Double IMUErr = 0.1176;

	private static Double CompassErr = (0.8) * (Math.PI / 180); // Rads now

	private static Double OdoErr = 0.0001;

	private static Integer IMURate = 10;

	private static Integer GPSRate = 40;

	private static Integer CompassRate = 125;

	private static Double RobotFrameVel = 0.002;// 0.0008;

	private static Double RobotFrameRot = 0.0001; // in Rads

	private static Double BridgeEast = 70.0; // East end of the bridge

	private static Double BridgeWest = 80.0; // West end of the bridge

	private Double GPSX, GPSY, IMUAccelX, IMUAccelY, IMUAccelZ, HeadingX, HeadingY, Heading, XOdo, YOdo, Theta, OldFrameVel, OldFrameRot, OldActualX, OldActualY, OldActualHeading, ActualSpeed;

	private Double ActualX, ActualY, ActualHeading;

	private Random r = new Random();

	private Integer StepCalc;

	private Boolean Simple, Stop, Go, Start, Aim;

	private Vector<Point> Points;

	private Point New;

	private Integer PointCounter, NumPoints;

	public RunMaker(boolean GPSMalfun, boolean IMUMalfun, boolean CompassMalfun, boolean OdoMalfun, boolean GPSBridge, Integer RunDuration) {
		GPSMal = GPSMalfun;
		IMUMal = IMUMalfun;
		CompassMal = CompassMalfun;
		OdoMal = OdoMalfun;
		GPSBrid = GPSBridge;
		RunLength = RunDuration;
		CurrentRun = 0;
		GPSX = 100.0;
		GPSY = 100.0;
		IMUAccelX = 0.0;
		IMUAccelY = 0.0;
		IMUAccelZ = 0.0;
		HeadingX = 0.0;
		HeadingY = 0.0;
		Heading = Math.PI / 2; // Facing N
		XOdo = 0.0;
		YOdo = 0.0;
		Theta = 0.0;
		// OldFrameVel = 0.0;
		// OldFrameRot = 0.0;
		OldActualX = ActualX = 100.0;
		OldActualY = ActualY = 100.0;
		OldActualHeading = ActualHeading = Math.PI / 2; // Facing N
		OutputLock = false;
		DLock = false;
		CLock = false;
		Finished = false;
		Updated = false;
		StepCalc = 0;
		Simple = true;
	}

	public RunMaker(boolean GPSMalfun, boolean IMUMalfun, boolean CompassMalfun, boolean OdoMalfun, boolean GPSBridge, Vector<Point> FreshPoints) {
		GPSMal = GPSMalfun;
		IMUMal = IMUMalfun;
		CompassMal = CompassMalfun;
		OdoMal = OdoMalfun;
		GPSBrid = GPSBridge;
		// RunLength = RunDuration;
		CurrentRun = 0;
		GPSX = 500.0;
		GPSY = 500.0;
		IMUAccelX = 0.0;
		IMUAccelY = 0.0;
		IMUAccelZ = 0.0;
		HeadingX = 0.0;
		HeadingY = 0.0;
		Heading = Math.PI / 2; // Facing N
		XOdo = 0.0;
		YOdo = 0.0;
		Theta = 0.0;
		// OldFrameVel = 0.0;
		// OldFrameRot = 0.0;
		OldActualX = ActualX = 500.0;
		OldActualY = ActualY = 500.0;
		OldActualHeading = ActualHeading = Math.PI / 2; // Facing N
		ActualSpeed = 0.0;
		OutputLock = false;
		DLock = false;
		CLock = false;
		Finished = false;
		Updated = false;
		StepCalc = 0;
		Simple = false;
		Points = FreshPoints;
		PointCounter = 0;
		NumPoints = Points.size();
		Start = true;
		Aim = false;
		Stop = false;
		Go = false;
	}

	public void step() {
		if (DLock == false && CLock == false) {
			StepLock = true;
			CurrentRun += TimestepSize;
			StepCalc += TimestepSize;
			if (Simple) { // Check type of calculation
				if (CurrentRun < RunLength) {

					// Do a step for Simple Ellipse method
					Double RFrameVel = RobotFrameVel;
					Double RFrameRot = RobotFrameRot;

					if (CurrentRun <= 1000) { // First second, come to
						// speed...
						RFrameVel = RFrameVel * (CurrentRun / 1000);
						RFrameRot = RFrameRot * (CurrentRun / 1000);
					}

					// Generate Actual Coordinates
					ActualHeading += RFrameRot;
					if (ActualHeading >= Math.PI) {
						ActualHeading -= (2 * Math.PI);
					}
					ActualX += RFrameVel * Math.cos(ActualHeading);
					ActualY += RFrameVel * Math.sin(ActualHeading);
				} else {
					// We're done!
					Finished = true;
					StepLock = false;
				}
			} else if (PointCounter < NumPoints) {
				// Do a step for Complex point to point method
				Double TurningSpeed = RobotFrameRot * 10;
				// Old = new Point(OldActualX, OldActualY);
				New = new Point(ActualX, ActualY);
				if (Stop == true && ActualSpeed > 0.0) {
					// Stop Case
					ActualSpeed = ActualSpeed / 2.0;
					if (ActualSpeed < 0.0001) {
						ActualSpeed = 0.0;
						Stop = false;
						Start = true;
					}
				} else if (Go == true) {
					if (ActualSpeed < RobotFrameVel)
						ActualSpeed += RobotFrameVel / 1000;
					// Check to see if we've achieved our point
					if (Points.get(PointCounter).distTo(New) < 1) {
						Stop = true;
						Go = false;
						PointCounter++;
					}
					
					//System.out.println("Go Diff: " + Points.get(PointCounter).distTo(New));

				} else if (Aim == true) {
					// Calculate if we're facing the right direction
					Double CurrentHeading = New.getTheta(Points.get(PointCounter));
					if ((CurrentHeading - ActualHeading) == 0.0) {
						Go = true;
						Aim = false;
						System.out.println("And now we're going!");
					} else {
						if (CurrentHeading < ActualHeading) {
							if (ActualHeading - CurrentHeading >= TurningSpeed)
								ActualHeading -= TurningSpeed;
							else
								ActualHeading = CurrentHeading;
						} else {
							if (CurrentHeading - ActualHeading >= TurningSpeed)
								ActualHeading += TurningSpeed;
							else
								ActualHeading = CurrentHeading;
						}

						if (ActualHeading >= Math.PI)
							ActualHeading -= (2 * Math.PI);
						if (ActualHeading < -Math.PI)
							ActualHeading += (2 * Math.PI);
					}
					//System.out.println("Heading Diff: " + (CurrentHeading - ActualHeading));

				} else {
					// Start must be true
					if (!Start)
						System.out.println("Error!  How'd we end up here?");

					if (Start) {
						System.out.println("Aiming at new point!");
						Start = false;
						Aim = true;
					}
				}

				// Made our Speed Choices, etc - update Actuals.
				ActualX += ActualSpeed * Math.cos(ActualHeading);
				ActualY += ActualSpeed * Math.sin(ActualHeading);
				//System.out.println("State: Start, Aim, Go, Stop - " + Start + ", " + Aim + ", " + Go + ", " + Stop);
				// System.out.println("Speed, Heading, X and Y: "+ ActualSpeed +
				// ", " + ActualHeading + ", "+ ActualX + ", " + ActualY);

			} else {
				// We're done!
				Finished = true;
				StepLock = false;
			}

			if (CurrentRun % IMURate == 0 && !Finished) {
				// IMUUpdate!

				Double XAcc = (ActualX - OldActualX) / Math.pow((((double) IMURate) / 1000), 2);
				Double YAcc = (ActualY - OldActualY) / Math.pow((((double) IMURate) / 1000), 2);
				XAcc += (r.nextInt(2) * (-2) + 1) * (r.nextDouble() * IMUErr);
				YAcc += (r.nextInt(2) * (-2) + 1) * (r.nextDouble() * IMUErr);
				// Check for malfunction
				if (IMUMal) {
					// Possible to Malfunction
					if (r.nextInt(MalfunProb) == 1) {
						// Oh great, now you've done it!
						System.out.println("IMU ERROR");
						XAcc = (r.nextInt(2) * (-2) + 1) * r.nextDouble() * 10;
						YAcc = (r.nextInt(2) * (-2) + 1) * r.nextDouble() * 10;
					}
				}

				IMUAccelX = XAcc;
				IMUAccelY = YAcc;
				IMUAccelZ = 0.0;
				Updated = true;
			}

			if (CurrentRun % GPSRate == 0 && !Finished) {
				// GPSUpdate!
				if (r.nextBoolean())
					GPSX = ActualX + (r.nextDouble() * GPSErr);
				else
					GPSX = ActualX - (r.nextDouble() * GPSErr);
				// System.out.println("Actual X vs. GPSX");
				// System.out.println(ActualX);
				// System.out.println(GPSX);
				// System.out.println(GPSX - ActualX);
				if (r.nextBoolean())
					GPSY = ActualY + (r.nextDouble() * GPSErr);
				else
					GPSY = ActualY - (r.nextDouble() * GPSErr);
				// System.out.println("Actual Y vs. GPSY");
				// System.out.println(ActualY);
				// System.out.println(GPSY);
				// System.out.println(GPSY - ActualY);
				// check for malfunction
				if (GPSBrid) {
					// We have a bridge! Add multipath errors if under the
					// bridge
					if (ActualX < BridgeEast && ActualX > BridgeWest) {
						// Under bridge!
						//System.out.println("Under the Bridge");
						Double BridgeDiff = BridgeEast - BridgeWest;
						if (r.nextBoolean())
							GPSX = GPSX + (r.nextDouble() * BridgeDiff);
						else
							GPSX = GPSX - (r.nextDouble() * BridgeDiff);

						if (r.nextBoolean())
							GPSY = GPSY + (r.nextDouble() * BridgeDiff);
						else
							GPSY = GPSY - (r.nextDouble() * BridgeDiff);
					}
				}

				if (GPSMal) {
					// Possible?
					if (r.nextInt(MalfunProb) == 1) {
						// Jeez...
						System.out.println("GPS ERROR");
						
						GPSX = 100 + (r.nextGaussian() * 80);
						GPSY = 100 + (r.nextGaussian() * 80);
					}
				}

				Updated = true;
			}

			if (CurrentRun % CompassErr == 0 && !Finished) {
				// CompassUpdate!
				Heading = ActualHeading + (r.nextInt(2) * (-2) + 1) * (r.nextGaussian() * CompassErr);
				if (CompassMal) {
					// Possible?
					if (r.nextInt(MalfunProb) == 1) {
						// Omg
						System.out.println("COMPASS ERROR");
						Heading = Math.PI / 2;
					}
				}

				if (Heading >= Math.PI) {
					Heading -= (2 * Math.PI);
				}

				Updated = true;
			}

			XOdo += (ActualX - OldActualX) + (r.nextInt(2) * (-1)) * (OdoErr) * (ActualX - OldActualX); //
			YOdo += (ActualY - OldActualY) + (r.nextInt(2) * (-1)) * (OdoErr) * (ActualY - OldActualY);
			Theta += (ActualHeading - OldActualHeading) + (r.nextInt(2) * (-2) + 1) * (0.001) * (ActualHeading - OldActualHeading);

			// OldFrameVel = RFrameVel;
			// OldFrameRot = RFrameRot;
			OldActualX = ActualX;
			OldActualY = ActualY;
			OldActualHeading = ActualHeading;

			StepLock = false;
		}

	}

	// Otherwise, wait for the Locks to clear!

	public boolean Finished() {
		if (Finished)
			System.out.println("Time Elapsed in Seconds: " + CurrentRun/1000);
		
		return Finished;
	}

	public boolean updated() {
		if (Updated) {
			Updated = false;
			DLock = true;
			CLock = true;
			return !Updated;
		}
		return Updated;
	}

	public String getD() {
		// Get a Data String
		String Data = "";
		// ("timestamp, elapsedtime, IMUAccelX, IMUAccelY, IMUAccelZ, HeadingX,
		// HeadingY, HeadingActual, unused, unused, unused, XOdo, YOdo, Theta,
		// GPSY, GPSX");
		Data = Data + ((double) StepCalc) / 1000;
		Data = Data + sDataSpace;
		Data = Data + ((double) CurrentRun) / 1000;
		Data = Data + sDataSpace;
		Data = Data + IMUAccelX;
		Data = Data + sDataSpace;
		Data = Data + IMUAccelY;
		Data = Data + sDataSpace;
		Data = Data + IMUAccelZ;
		Data = Data + sDataSpace;
		Data = Data + HeadingX;
		Data = Data + sDataSpace;
		Data = Data + HeadingY;
		Data = Data + sDataSpace;
		Data = Data + Heading;
		Data = Data + sDataSpace;
		Data = Data + "0";
		Data = Data + sDataSpace;
		Data = Data + "0";
		Data = Data + sDataSpace;
		Data = Data + "0";
		Data = Data + sDataSpace;
		Data = Data + XOdo;
		Data = Data + sDataSpace;
		Data = Data + YOdo;
		Data = Data + sDataSpace;
		Data = Data + Theta;
		Data = Data + sDataSpace;
		Data = Data + GPSY;
		Data = Data + sDataSpace;
		Data = Data + GPSX;
		DLock = false;
		if (CLock == false)
			StepCalc = 0;
		return Data;
	}

	public String getC() {
		// Get a Control String
		String Control = "";
		// ("timestamp, elapsedtime, ActualX, ActualY, Actual Heading");
		Control = Control + ((double) StepCalc) / 1000;
		Control = Control + sDataSpace;
		Control = Control + ((double) CurrentRun) / 1000;
		Control = Control + sDataSpace;
		Control = Control + ActualX;
		Control = Control + sDataSpace;
		Control = Control + ActualY;
		Control = Control + sDataSpace;
		Control = Control + ActualHeading;
		CLock = false;
		if (DLock == false)
			StepCalc = 0;

		return Control;
	}

}
