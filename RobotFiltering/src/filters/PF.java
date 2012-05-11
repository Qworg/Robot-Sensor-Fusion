package filters;

import java.io.*;
import java.util.Date;
import java.util.Random;
import java.util.Vector;

import Jama.EigenvalueDecomposition;
import Jama.Matrix;

import testsystem.CompassEntry;
import testsystem.Data;
import testsystem.GPSEntry;
import testsystem.IMUEntry;
import testsystem.OdoEntry;
import controllers.*;

public class PF implements Filter {

	private boolean ProcessOn = false;

	private boolean ProcessDone = false;

	private Date TimeOn, TimeOff;

	private Long TimeElapsed;

	private Data dDataset;

	private Control cController;

	private EKFControl cEController;

	private SPKFControl cUController;

	private int NumParticles;

	private int NumMixParticles;

	private Vector<GPSEntry> Output = new Vector<GPSEntry>();

	private Vector<Matrix> Resid = new Vector<Matrix>();

	protected Random rand = new Random(new Date().getTime());

	static int L = 9;

	private int CoreType;

	private Double dR, dQ;

	private Double defA, defB, defK;

	private int NumKLD;

	private boolean ResampleStep = false;

	public PF(Control Controller, Data Dataset, int Particles, Control DController, int FilterType, Double initQ, Double initR) {
		cController = Controller;
		dDataset = Dataset;
		NumParticles = Particles;
		NumMixParticles = (int) (NumParticles / 20);
		NumKLD = 0;
		dR = initR;
		dQ = initQ;
		switch (FilterType) {
		case 1:
			// Standard PF
			CoreType = 1; // Filter Type @ core [with UKF working]
			cUController = (SPKFControl) DController;
			this.setAlpha(0.02);
			this.setBeta(2.0);
			this.setKappa(0.0);
			break;
		case 2:
			// EKF PF Types
			CoreType = 2; // Filter Type @ core - EKF! [EKF working]
			cEController = (EKFControl) DController;
			break;
		case 3:
			// UKF PF Types
			CoreType = 3; // Filter Type @ core - UKF! [UKF working]
			cUController = (SPKFControl) DController;
			this.setAlpha(0.02);
			this.setBeta(2.0);
			this.setKappa(0.0);
			break;
		default:
			System.out.println("Obviously, something is screwed up!");
			break;
		}

	}

	public PF(Control Controller, Data Dataset, int Particles, Control DController, int FilterType, Double initQ, Double initR, Double dA, Double dB, Double dK) {
		cController = Controller;
		dDataset = Dataset;
		NumParticles = Particles;
		NumMixParticles = (int) (NumParticles / 20);
		dR = initR;
		dQ = initQ;
		switch (FilterType) {
		case 1:
			// Standard PF
			CoreType = 1; // Filter Type @ core [with UKF working]
			cUController = (SPKFControl) DController;
			this.setAlpha(dA);
			this.setBeta(dB);
			this.setKappa(dK);
			break;
		case 2:
			// EKF PF Types
			CoreType = 2; // Filter Type @ core - EKF! [EKF working]
			System.out.println("You sure you didn't want a UKF-PF?");
			cEController = (EKFControl) DController;
			break;
		case 3:
			// UKF PF Types
			CoreType = 3; // Filter Type @ core - UKF! [UKF working]
			cUController = (SPKFControl) DController;
			this.setAlpha(dA);
			this.setBeta(dB);
			this.setKappa(dK);
			break;
		default:
			System.out.println("Obviously, something is screwed up!");
			break;
		}

	}

	public void setAlpha(Double Alpha) {
		if (!ProcessOn) {
			// Change Alpha
			cUController.setAlpha(Alpha);
			defA = Alpha;
		} else
			System.out.println("Can't reset Alpha in mid-process!");
	}

	public void setBeta(Double Beta) {
		if (!ProcessOn) {
			// Change Beta
			cUController.setBeta(Beta);
			defB = Beta;
		} else
			System.out.println("Can't reset Beta in mid-process!");
	}

	public void setKappa(Double Kappa) {
		if (!ProcessOn) {
			// Change Kappa
			cUController.setKappa(Kappa);
			defK = Kappa;
		} else
			System.out.println("Can't reset Kappa in mid-process!");
	}

	public Boolean finishedProcess() {
		return ProcessDone;
	}

	public void outputData(String FileName) {
		try {
			BufferedWriter out = new BufferedWriter(new FileWriter(FileName + TimeElapsed + ".txt"));
			int len = Output.size();
			System.out.println("Time Elapsed: " + TimeElapsed);
			GPSEntry A;
			Matrix B;
			out.write("#Timestep EastX NorthY ResidX ResidY ResidZ");
			out.newLine();
			for (int i = 0; i < len; i++) {
				A = Output.elementAt(i);
				B = Resid.elementAt(i);
				out.write(A.getGPSTimestamp() + " " + A.getEastX() + " " + A.getNorthY() + " " + B.get(0, 0) + " " + B.get(1, 0) + " " + B.get(2, 0));
				out.newLine();
			}
			out.close();
		} catch (IOException e) {
			System.out.println("Output in PF Blew Up");
			e.printStackTrace();
		}

	}

	public void setControl(Control Controller) {
		if (!ProcessOn) {
			// Change the Controller
			cController = Controller;
		} else
			System.out.println("Can't reset controller in mid-process!");
	}

	public void setData(Data Dataset) {
		if (!ProcessOn) {
			// Change the Dataset
			dDataset = Dataset;
		} else
			System.out.println("Can't reset data in mid-process!");
	}

	public void startProcess() {
		// Generate new Variables
		GPSEntry gGPS;
		GPSEntry gOldGPS;
		IMUEntry iIMU;
		OdoEntry oOdo;
		CompassEntry cComp;
		Double dTS;
		Vector<Double> vIMU = new Vector<Double>();
		Vector<Double> vComp = new Vector<Double>();
		Vector<Double> DeltaH = new Vector<Double>();
		Vector<Double> tempOdo = new Vector<Double>();
		Vector<Double> vGPS = new Vector<Double>();
		Vector<Double> vMove = new Vector<Double>();
		Vector<Matrix> mParticleSet = new Vector<Matrix>();
		Vector<Matrix> mixParticleSet = new Vector<Matrix>();
		Vector<Double> vGreekHeading = new Vector<Double>();
		Vector<Double> vGreekHeading1 = new Vector<Double>();
		Vector<Double> vGreekDrive = new Vector<Double>();
		Vector<Double> vGreekPose = new Vector<Double>();

		// Start the process!
		ProcessOn = true;
		TimeOn = new Date();
		// Get the length of the dataset
		int iLength = dDataset.getLength();

		// Build the First States
		vGreekHeading.add(defA);
		vGreekHeading.add(defB);
		vGreekHeading.add(defK);
		vGreekDrive = vGreekPose = vGreekHeading1 = vGreekHeading;

		Matrix HeadingX = new Matrix(9, 1, 0.0);
		Matrix HeadingP = new Matrix(9, 9, 0.0);
		Matrix Heading1X = new Matrix(9, 1, 0.0);
		Matrix Heading1P = new Matrix(9, 9, 0.0);
		Matrix DriveX = new Matrix(9, 1, 0.0);
		Matrix DriveP = new Matrix(9, 9, 0.0);
		Matrix PoseX = new Matrix(9, 1, 0.0);
		Matrix PoseP = new Matrix(9, 9, 0.0);
		// Set Kalman
		Matrix ZeroHeadingK = new Matrix(6, 3, 0.0);
		Matrix HeadingK = new Matrix(6, 3, 0.0);
		Matrix DriveK = new Matrix(6, 3, 0.0);
		Matrix PoseK = new Matrix(6, 3, 0.0);

		// Reset the P values to norm
		HeadingP.set(3, 3, dQ);
		HeadingP.set(4, 4, dQ);
		HeadingP.set(5, 5, dQ);
		HeadingP.set(6, 6, dR);
		HeadingP.set(7, 7, dR);
		HeadingP.set(8, 8, dR);

		Heading1P.set(3, 3, dQ);
		Heading1P.set(4, 4, dQ);
		Heading1P.set(5, 5, dQ);
		Heading1P.set(6, 6, dR);
		Heading1P.set(7, 7, dR);
		Heading1P.set(8, 8, dR);

		DriveP.set(3, 3, dQ);
		DriveP.set(4, 4, dQ);
		DriveP.set(5, 5, dQ);
		DriveP.set(6, 6, dR);
		DriveP.set(7, 7, dR);
		DriveP.set(8, 8, dR);

		PoseP.set(3, 3, dQ);
		PoseP.set(4, 4, dQ);
		PoseP.set(5, 5, dQ);
		PoseP.set(6, 6, dR);
		PoseP.set(7, 7, dR);
		PoseP.set(8, 8, dR);

		gGPS = dDataset.GetGPSEntry(0);

		Matrix tQ = new Matrix(1, 3, dQ);
		Matrix tR = new Matrix(1, 3, dR);
		// Set Kalman
		ZeroHeadingK.setMatrix(3, 3, 0, 2, tQ);
		ZeroHeadingK.setMatrix(4, 4, 0, 2, tR);
		HeadingK.setMatrix(3, 3, 0, 2, tQ);
		HeadingK.setMatrix(4, 4, 0, 2, tR);
		DriveK.setMatrix(3, 3, 0, 2, tQ);
		DriveK.setMatrix(4, 4, 0, 2, tR);
		PoseK.setMatrix(3, 3, 0, 2, tQ);
		PoseK.setMatrix(4, 4, 0, 2, tR);

		// Set first position and build the particle set

		gGPS = dDataset.GetGPSEntry(0);
		gOldGPS = gGPS;
		Double startX = gGPS.getEastX();
		Double startY = gGPS.getNorthY();
		Heading1X.set(2, 0, dDataset.GetOdoEntry(0).getOdoTheta());
		HeadingX.set(2, 0, dDataset.GetCompassEntry(0).GetRad());
		DriveX.set(0, 0, dDataset.GetOdoEntry(0).getOdoX());
		DriveX.set(1, 0, dDataset.GetOdoEntry(0).getOdoY());
		PoseX.set(0, 0, startX);
		PoseX.set(1, 0, startY);
		PoseK.set(0, 0, startX);
		PoseK.set(0, 1, startY);

		// Particles will start within a Radius meter radius of the GPS spot -
		// randomly distributed
		Double Radius = 25.0;
		Matrix tempParticle = new Matrix(3, 1);

		// DEBUG: Header for Regular Particles
		System.out.println("Regular Particle Set: ");
		// END DEBUG
		for (int i = 0; i < NumParticles; i++) {
			tempParticle = new Matrix(3, 1, 0.0);
			Double RandRadius = ((Radius * rand.nextDouble()) * (-1 * rand.nextInt(2)));
			// System.out.println("Rand Radius = "+RandRadius);
			tempParticle.set(0, 0, startX + RandRadius);
			// System.out.print(startX+RandRadius+",");
			RandRadius = ((Radius * rand.nextDouble()) * (-1 * rand.nextInt(2)));
			// System.out.println("Rand Radius = "+RandRadius);
			tempParticle.set(1, 0, startY + RandRadius);
			// System.out.print(startY+RandRadius+",");
			//RandRadius = ((Radius * rand.nextDouble()) * (-1 * rand.nextInt(2)));
			// System.out.println("Rand Radius = "+RandRadius);
			tempParticle.set(2, 0, 0.0);
			// DEBUG: Particle List Post update
			tempParticle.print(3, 2);
			// END DEBUG
			mParticleSet.add(tempParticle);
		}
		// DEBUG: Header for Mix Particles
		System.out.println("Mix Particle Set: ");
		// END DEBUG
		for (int i = 0; i < NumMixParticles; i++) {
			tempParticle = new Matrix(3, 1);
			Double RandRadius = ((Radius * rand.nextDouble()) * (-1 * rand.nextInt(2)));
			// System.out.println("Rand Radius = "+RandRadius);
			tempParticle.set(0, 0, startX + RandRadius);
			RandRadius = ((Radius * rand.nextDouble()) * (-1 * rand.nextInt(2)));
			// System.out.println("Rand Radius = "+RandRadius);
			tempParticle.set(1, 0, startY + RandRadius);
			//RandRadius = ((Radius * rand.nextDouble()) * (-1 * rand.nextInt(2)));
			// System.out.println("Rand Radius = "+RandRadius);
			tempParticle.set(2, 0, 0.0);
			// DEBUG: Particle List Post update
			tempParticle.print(3, 2);
			// END DEBUG
			mixParticleSet.add(tempParticle);
		}

		// //ALTERNATIVE - Particles will start within a Radius meter radius of
		// the GPS spot - Gaussian distribution
		// Double Radius = 10.0;
		// Matrix tempParticle = new Matrix(1, 3);
		// for (int i = 0; i < NumParticles; i++) {
		// tempParticle.set(0, 0,
		// startX+((Radius*rand.nextGaussian())*(-1*rand.nextInt(2))));
		// tempParticle.set(0, 1,
		// startY+((Radius*rand.nextGaussian())*(-1*rand.nextInt(2))));
		// tempParticle.set(0, 2,
		// (Radius*rand.nextDouble())*(-1*rand.nextInt(2)));
		// mParticleSet.add(tempParticle);
		// }

		// SPKFStep HeadingU, Heading1U, DriveU, PoseU;
		Matrix TotalMove = new Matrix(3, 1, 0.0);
		Double TotalWeight = 0.0;
		Double LargestWeight = 0.0;
		Vector<Double> Weights = new Vector<Double>();
		Vector<Double> TrueWeights = new Vector<Double>();
		Double Weight;

		Matrix tempResid = new Matrix(3, 1, 0.0);

		Double OldTheta = 0.0;
		Double OldOdoX = 0.0;
		Double OldOdoY = 0.0;
		// Start the roll!
		// DEBUG
		System.out.println("CoreType of PF: " + CoreType);
		// END DEBUG
		for (int i = 1; i < iLength; i++) {
			// Get the data @ i
			gGPS = dDataset.GetGPSEntry(i);
			iIMU = dDataset.GetIMUEntry(i);
			oOdo = dDataset.GetOdoEntry(i);
			cComp = dDataset.GetCompassEntry(i);
			dTS = gGPS.getGPSTimestamp();

			if (CoreType == 1 || CoreType == 3) {
				// Step 1:
				tempOdo.clear();
				tempOdo.add(0.0);
				tempOdo.add(0.0);
				tempOdo.add(oOdo.getOdoTheta() - OldTheta);
				OldTheta = oOdo.getOdoTheta();
				vIMU.clear();
				vIMU.add(0.0);
				vIMU.add(0.0);
				vIMU.add(iIMU.getIMURotAccel().get(2) * dTS);

				SPKFStep Heading1U = new SPKFStep(Heading1P, Heading1X, vIMU, tempOdo, vGreekHeading1, dTS);
				while (!Heading1U.StepFinished()) {
					try {
						Thread.sleep(10);
					} catch (InterruptedException e) {
						e.printStackTrace();
					}
				}
				Heading1P = Heading1U.getP();
				Heading1X = Heading1U.getX();
				vGreekHeading1 = Heading1U.getGreek();

				// Step 2:
				vComp.clear();
				vComp.add(0.0);
				vComp.add(0.0);
				vComp.add(cComp.GetRad());

				tempOdo.clear();
				tempOdo.add(Heading1X.get(0, 0));
				tempOdo.add(Heading1X.get(1, 0));
				tempOdo.add(Heading1X.get(2, 0));

				SPKFStep HeadingU = new SPKFStep(HeadingP, HeadingX, tempOdo, vComp, vGreekHeading, dTS);
				while (!HeadingU.StepFinished()) {
					try {
						Thread.sleep(10);
					} catch (InterruptedException e) {
						e.printStackTrace();
					}
				}
				HeadingP = HeadingU.getP();
				HeadingX = HeadingU.getX();
				vGreekHeading = HeadingU.getGreek();

				DeltaH.clear();
				DeltaH.add(HeadingX.get(0, 0));
				DeltaH.add(HeadingX.get(1, 0));
				DeltaH.add(HeadingX.get(2, 0));

				// Step 3:
				vIMU = dDataset.GetIMUVelocity(i, DeltaH);
				for (int j = 0; j < vIMU.size(); j++) {
					// //DEBUG: What is vIMU?
					// System.out.println(vIMU.get(j));
					vIMU.set(j, (vIMU.get(j) * dTS));
					// //DEBUG: What is vIMU?
					// System.out.println(vIMU.get(j));
				}
				// This yields Velocity -^

				// Step 3:
				tempOdo.clear();
				tempOdo.add(oOdo.getOdoX() - OldOdoX);
				tempOdo.add(oOdo.getOdoY() - OldOdoY);
				tempOdo.add(0.0);
				OldOdoX = oOdo.getOdoX();
				OldOdoY = oOdo.getOdoY();

				SPKFStep DriveU = new SPKFStep(DriveP, DriveX, vIMU, tempOdo, vGreekDrive, dTS);
				while (!DriveU.StepFinished()) {
					try {
						Thread.sleep(10);
					} catch (InterruptedException e) {
						e.printStackTrace();
					}
				}
				DriveP = DriveU.getP();
				DriveX = DriveU.getX();
				vGreekDrive = DriveU.getGreek();

				TotalMove = TotalMove.plus(DriveX.getMatrix(0, 2, 0, 0));

				// //DEBUG: MOVE
				// System.out.println("Drive in SPKF: " + DriveX.get(0, 0) +
				// ","+ DriveX.get(1, 0) + ","+ DriveX.get(2, 0));
				// System.out.println("Total Move: " + TotalMove.get(0, 0) + ","
				// + TotalMove.get(1, 0) + "," + TotalMove.get(2, 0));
				// //TotalMove.print(3, 2);
				// //DriveX.getMatrix(0, 2, 0, 0).print(3, 2);
				// //END DEBUG

				for (int m = 0; m < NumParticles; m++) {
					tempParticle = mParticleSet.get(m);
					// System.out.println("PreTempParticle: " +
					// tempParticle.get(0, 0) + "," +tempParticle.get(1, 0));
					tempParticle = tempParticle.plus(DriveX.getMatrix(0, 2, 0, 0));
					tempParticle.set(0, 0, tempParticle.get(0, 0) + (rand.nextGaussian() * (-1 * rand.nextInt(2))));
					tempParticle.set(1, 0, tempParticle.get(1, 0) + (rand.nextGaussian() * (-1 * rand.nextInt(2))));
					// System.out.println("PostTempParticle: " +
					// tempParticle.get(0, 0) + "," +tempParticle.get(1, 0));
					mParticleSet.set(m, tempParticle);
				}
			} else {

				// Core Type 2!
				// Step 0:
				vIMU.clear();
				vIMU.add(0.0);
				vIMU.add(0.0);
				vIMU.add(iIMU.getIMURotAccel().get(2) * dTS);
				tempOdo.clear();
				tempOdo.add(0.0);
				tempOdo.add(0.0);
				tempOdo.add((oOdo.getOdoTheta() - OldTheta));
				OldTheta = oOdo.getOdoTheta();

				ZeroHeadingK = KStep(ZeroHeadingK, tempOdo, vIMU, dTS);

				// Step 1:
				tempOdo.clear();
				tempOdo.add(ZeroHeadingK.get(0, 0));
				tempOdo.add(ZeroHeadingK.get(0, 1));
				tempOdo.add(ZeroHeadingK.get(0, 2));

				vComp.clear();
				vComp.add(0.0);
				vComp.add(0.0);
				vComp.add(cComp.GetRad());

				HeadingK = KStep(HeadingK, vComp, tempOdo, dTS);

				DeltaH.add(HeadingK.get(0, 0));
				DeltaH.add(HeadingK.get(0, 1));
				DeltaH.add(HeadingK.get(0, 2));

				// Step 2:
				vIMU = dDataset.GetIMUVelocity(i, DeltaH);
				for (int j = 0; j < vIMU.size(); j++) {
					vIMU.set(j, (vIMU.get(j) * dTS));
				}

				// Step 3:
				tempOdo.clear();
				tempOdo.add(oOdo.getOdoX());
				tempOdo.add(oOdo.getOdoY());
				tempOdo.add(0.0);

				DriveK = KStep(DriveK, tempOdo, vIMU, dTS);

				TotalMove = TotalMove.plus(DriveK.getMatrix(0, 0, 0, 2).transpose());

				// //DEBUG: MOVE
				// System.out.println("Drive in EKF: ");
				// DriveX.getMatrix(0, 0, 0, 2).transpose().print(3, 2);
				// //END DEBUG

				for (int m = 0; m < NumParticles; m++) {
					tempParticle = mParticleSet.get(m);

					tempParticle = tempParticle.plus(DriveK.getMatrix(0, 0, 0, 2).transpose());
					tempParticle.set(0, 0, tempParticle.get(0, 0) + (rand.nextGaussian() * (-1 * rand.nextInt(2))));
					tempParticle.set(1, 0, tempParticle.get(1, 0) + (rand.nextGaussian() * (-1 * rand.nextInt(2))));

					mParticleSet.set(m, tempParticle);
				}

			}
			// Resample the Mix Particles using Traditional PF
			Double mixGPSX = gOldGPS.getEastX() + TotalMove.get(0, 0);
			Double mixGPSY = gOldGPS.getNorthY() + TotalMove.get(1, 0);
			Double mixGPSZ = 0.0 + TotalMove.get(2, 0);
			TotalWeight = 0.0;
			LargestWeight = 0.0;
			TrueWeights.clear();
			Weights.clear();

			for (int m = 0; m < NumMixParticles; m++) {
				// For each particle, calculate distance from GPS
				// heading and weight accordingly. Keep track of total
				// weight.
				tempParticle = mixParticleSet.get(m);
				Double TempX = tempParticle.get(0, 0) - mixGPSX;
				Double TempY = tempParticle.get(1, 0) - mixGPSY;
				Double TempZ = tempParticle.get(2, 0) - mixGPSZ;
				Weight = Math.sqrt((TempX * TempX) + (TempY * TempY) + (TempZ * TempZ));
				Weights.add(Weight);
				TotalWeight += Weight;
				if (LargestWeight < Weight)
					LargestWeight = Weight;
			}

			TotalWeight = (LargestWeight * NumMixParticles) - TotalWeight;

			for (int m = 0; m < NumMixParticles; m++) {
				Weight = (LargestWeight - Weights.get(m))/TotalWeight;
				Weights.set(m, Weight);
			}

			mixParticleSet = WRS(mixParticleSet, Weights, TotalWeight);

			// //DEBUG
			// System.out.println("OldGPS = " + gOldGPS.getEastX() + " , " +
			// gOldGPS.getNorthY());
			// System.out.println("GPS = " + gGPS.getEastX() + " , " +
			// gGPS.getNorthY());
			// //ENDDEBUG

			// Particle driving done! Check to see if GPS updated.
			double OldEast = gOldGPS.getEastX();
			double NewEast = gGPS.getEastX();
			double OldNorth = gOldGPS.getNorthY();
			double NewNorth = gGPS.getNorthY();
			// if (gOldGPS.getEastX() != gGPS.getEastX() || gOldGPS.getNorthY()
			// != gGPS.getNorthY()) {
			if (OldEast != NewEast || OldNorth != NewNorth) {
				// GPS WAS UPDATED!
				// Time for a Crime Spree - or better yet, a Resampling Step
				// (this is where MAGIC unfolds)
				ResampleStep = true;

				// //DEBUG
				// System.out.println("Resample!");
				// System.out.println("Total Move: " + TotalMove.get(0,0) + ","
				// + TotalMove.get(1, 0));
				// System.out.println("GPS Move: " + (NewEast-OldEast) + "," +
				// (NewNorth-OldNorth));
				// //ENDDEBUG

				Double GPSX = 0.0;
				Double GPSY = 0.0;
				Double GPSZ = 0.0;
				Matrix mixDriveSet = new Matrix(3, 1, 0.0);
				vGPS.clear();
				vGPS.add(gGPS.getEastX());
				vGPS.add(gGPS.getNorthY());
				vGPS.add(0.0);
				vMove.clear();
				vMove.add(TotalMove.get(0, 0));
				vMove.add(TotalMove.get(1, 0));
				vMove.add(TotalMove.get(2, 0));
				double WeightMod = 0.001;
				TotalWeight = 0.0;
				LargestWeight = 0.0;
				Weights.clear();
				TrueWeights.clear();

				if (CoreType == 1) {
					// Standard PF LAND!
					// Steps:
					// 1) Weight Particles
					// 2) Resample!
					// Easy, no?

					GPSX = gGPS.getEastX();
					GPSY = gGPS.getNorthY();

					Weights.clear();
					TrueWeights.clear();
					for (int m = 0; m < NumParticles; m++) {
						// For each particle, calculate distance from GPS
						// heading and weight accordingly. Keep track of total
						// weight.
						tempParticle = mParticleSet.get(m);
						// System.out.println("TempParticle for Weighting: ");
						// tempParticle.print(3, 2);
						Double TempX = tempParticle.get(0, 0) - GPSX;
						Double TempY = tempParticle.get(1, 0) - GPSY;
						Double TempZ = tempParticle.get(2, 0) - GPSZ;
						Weight = Math.sqrt((TempX * TempX) + (TempY * TempY) + (TempZ * TempZ));
						// System.out.println("Weight = "+Weight);
						Weights.add(Weight);
						TotalWeight += Weight;
						if (LargestWeight < Weight)
							LargestWeight = Weight;
					}

					TotalWeight = (LargestWeight * NumParticles) - TotalWeight + (WeightMod * NumParticles);
					// //DEBUG
					// System.out.println("TotalWeight: " + TotalWeight);
					// //ENDDEBUG

					for (int m = 0; m < NumParticles; m++) {
						Weight = (LargestWeight - Weights.get(m) + WeightMod)/TotalWeight;
						// // DEBUG
						// System.out.println("Adapted Weight @ " + m + ": " +
						// Weight);
						// // ENDDEBUG
						Weights.set(m, Weight);
					}

					// System.out.println("Total Weight");
					// // DEBUG
					// System.out.println("PreWRS");
					// for (int m = 0; m < NumParticles; m++) {
					// tempParticle = mParticleSet.get(m);
					// tempParticle.print(3, 2);
					// }
					// // ENDDEBUG
					mParticleSet = KLD(mParticleSet, gGPS, Weights, TotalWeight);
					// // DEBUG
					// System.out.println("PostWRS");
					// for (int m = 0; m < NumParticles; m++) {
					// tempParticle = mParticleSet.get(m);
					// tempParticle.print(3, 2);
					// }
					// // ENDDEBUG

					// Drive Mix Particles
					mixDriveSet.set(0, 0, GPSX - gOldGPS.getEastX());
					mixDriveSet.set(1, 0, GPSY - gOldGPS.getNorthY());
					mixDriveSet.set(2, 0, GPSZ);
					for (int m = 0; m < NumMixParticles; m++) {
						mixParticleSet.set(m, mixParticleSet.get(m).plus(mixDriveSet));
					}

				} else if (CoreType == 2) {
					// EKF PF LAND!
					// Steps:
					// 1) Combine old GPS + Total Move and Fuse with new GPS
					// 2) Weight Particles
					// 3) Resample!
					// No cryyyying.
					PoseK = KStep(PoseK, vGPS, vMove, dTS);

					// FUSION COMPLETE!
					GPSX = PoseK.get(0, 0);
					GPSY = PoseK.get(0, 1);
					GPSZ = PoseK.get(0, 2);

					Weights.clear();
					TrueWeights.clear();
					for (int m = 0; m < NumParticles; m++) {
						// For each particle, calculate distance from GPS
						// heading and weight accordingly. Keep track of total
						// weight.
						tempParticle = mParticleSet.get(m);
						// System.out.println("TempParticle for Weighting: ");
						// tempParticle.print(3, 2);
						Double TempX = tempParticle.get(0, 0) - GPSX;
						Double TempY = tempParticle.get(1, 0) - GPSY;
						Double TempZ = tempParticle.get(2, 0) - GPSZ;
						Weight = Math.sqrt((TempX * TempX) + (TempY * TempY) + (TempZ * TempZ));
						// System.out.println("Weight = "+Weight);
						Weights.add(Weight);
						TotalWeight += Weight;
						if (LargestWeight < Weight)
							LargestWeight = Weight;
					}

					TotalWeight = (LargestWeight * NumParticles) - TotalWeight + (WeightMod * NumParticles);
					// //DEBUG
					// System.out.println("TotalWeight: " + TotalWeight);
					// //ENDDEBUG

					for (int m = 0; m < NumParticles; m++) {
						Weight = (LargestWeight - Weights.get(m) + WeightMod)/TotalWeight;
//						 // DEBUG
//						 System.out.println("Adapted Weight @ " + m + ": " + Weight);
//						 // ENDDEBUG
						Weights.set(m, Weight);
					}

					mParticleSet = KLD(mParticleSet, gGPS, Weights, TotalWeight);

					// Drive Mix Particles
					mixDriveSet.set(0, 0, GPSX - gOldGPS.getEastX());
					mixDriveSet.set(1, 0, GPSY - gOldGPS.getNorthY());
					mixDriveSet.set(2, 0, GPSZ);

					for (int m = 0; m < NumMixParticles; m++) {
						mixParticleSet.set(m, mixParticleSet.get(m).plus(mixDriveSet));
					}

				} else if (CoreType == 3) {
					// Okay, so we GOTTA be in UKF Land? Right!?
					// Steps:
					// 1) Combine old GPS + Total Move and Fuse with new GPS
					// 2) Weight Particles
					// 3) Resample!
					SPKFStep PoseU = new SPKFStep(PoseP, PoseX, vMove, vGPS, vGreekPose, dTS);
					while (!PoseU.StepFinished()) {
						try {
							Thread.sleep(10);
						} catch (InterruptedException e) {
							e.printStackTrace();
						}
					}
					PoseP = PoseU.getP();
					PoseX = PoseU.getX();
					vGreekPose = PoseU.getGreek();

					// FUSION COMPLETE
					GPSX = PoseX.get(0, 0);
					GPSY = PoseX.get(1, 0);
					GPSZ = PoseX.get(2, 0);

					Weights.clear();
					TrueWeights.clear();
					for (int m = 0; m < NumParticles; m++) {
						// For each particle, calculate distance from GPS
						// heading and weight accordingly. Keep track of total
						// weight.
						tempParticle = mParticleSet.get(m);
						// System.out.println("TempParticle for Weighting: ");
						// tempParticle.print(3, 2);
						Double TempX = tempParticle.get(0, 0) - GPSX;
						Double TempY = tempParticle.get(1, 0) - GPSY;
						Double TempZ = tempParticle.get(2, 0) - GPSZ;
						Weight = Math.sqrt((TempX * TempX) + (TempY * TempY) + (TempZ * TempZ));
						// System.out.println("Weight = "+Weight);
						Weights.add(Weight);
						TotalWeight += Weight;
						if (LargestWeight < Weight)
							LargestWeight = Weight;
					}

					TotalWeight = (LargestWeight * NumParticles) - TotalWeight + (WeightMod * NumParticles);
					// //DEBUG
					// System.out.println("TotalWeight: " + TotalWeight);
					// //ENDDEBUG

					for (int m = 0; m < NumParticles; m++) {
						Weight = (LargestWeight - Weights.get(m) + WeightMod)/TotalWeight;
//						 // DEBUG
//						 System.out.println("Adapted Weight @ " + m + ": " + Weight);
//						 // ENDDEBUG
						Weights.set(m, Weight);
					}

					mParticleSet = KLD(mParticleSet, gGPS, Weights, TotalWeight);

					// Drive Mix Particles
					mixDriveSet.set(0, 0, GPSX - gOldGPS.getEastX());
					mixDriveSet.set(1, 0, GPSY - gOldGPS.getNorthY());
					mixDriveSet.set(2, 0, GPSZ);
					for (int m = 0; m < NumMixParticles; m++) {
						mixParticleSet.set(m, mixParticleSet.get(m).plus(mixDriveSet));
					}

				} else
					System.out.println("Jeff REALLY screwed up");

				gOldGPS = gGPS;
				TotalMove = new Matrix(3, 1, 0.0);
			}

			// Extract Actual Location!
			Double Xm = 0.0;
			Double Ym = 0.0;
			Double Zm = 0.0;
			for (int m = 0; m < NumParticles; m++) {
				// Mean method = Fastest
				tempParticle = mParticleSet.get(m);

				// // DEBUG: Particle List Post update
				// System.out.println("Actual Particle " + m + ": ");
				// tempParticle.print(3, 2);
				// // END DEBUG

				Xm += tempParticle.get(0, 0);//*Weights.get(m);
				Ym += tempParticle.get(1, 0);//*Weights.get(m);
				Zm += tempParticle.get(2, 0);//*Weights.get(m);
			}
			for (int m = 0; m < NumMixParticles; m++) {
				// Mean method = Fastest
				// GET THOSE MIX PARTICLES IN THERE!
				tempParticle = mixParticleSet.get(m);

				// // DEBUG: Particle List Post update
				// System.out.println("Mix Particle " + m + ": ");
				// tempParticle.print(3, 2);
				// // END DEBUG

				Xm += tempParticle.get(0, 0);
				Ym += tempParticle.get(1, 0);
				Zm += tempParticle.get(2, 0);
			}

			// System.out.println("Location Extraction PreDivide:" + Xm + "," +
			// Ym + "," + Zm);

//			 Xm = Xm / ((double) NumParticles);
//			 Ym = Ym / ((double) NumParticles);
//			 Zm = Zm / ((double) NumParticles);
			Xm = Xm / ((double) (NumParticles + NumMixParticles));
			Ym = Ym / ((double) (NumParticles + NumMixParticles));
			Zm = Zm / ((double) (NumParticles + NumMixParticles));
			GPSEntry GPSOut = new GPSEntry(Xm, Ym, dTS);
			Output.add(GPSOut);

			// System.out.println("Location Extraction PostDivide:" + Xm + "," +
			// Ym + "," + Zm);

			tempResid = new Matrix(3, 1, 0.0);
			tempResid.set(0, 0, Xm - gGPS.getEastX());
			tempResid.set(1, 0, Ym - gGPS.getNorthY());
			Resid.add(tempResid);

//			if (ResampleStep) {
//				// Particle Number Manipulation
//
//				// //DEBUG
//				// System.out.println("Number of Particles
//				// Pre:"+mParticleSet.size());
//				// //ENDDEBUG
//				mParticleSet = LikelyhoodBasedAdapt(mParticleSet, gGPS, GPSOut, Weights);
//				// DEBUG
//				System.out.println("Number of Particles Post:" + mParticleSet.size());
//				// ENDDEBUG
//				ResampleStep = false;
//			}

			// Ticker
			if (i % 1000 == 0)
				System.out.print(".");
		}

		// Process done!
		TimeOff = new Date();
		TimeElapsed = TimeOff.getTime() - TimeOn.getTime();
		ProcessOn = false;
		ProcessDone = true;

	}

	public long getTimeElapsed() {
		return TimeElapsed;
	}

	private Vector<Matrix> LikelyhoodBasedAdapt(Vector<Matrix> Incoming, GPSEntry GPSVal, GPSEntry CalcLoc, Vector<Double> iWeights) {
		int M = Incoming.size();
		Vector<Matrix> Outgoing = Incoming;

		double XOff = CalcLoc.getEastX() - GPSVal.getEastX();
		double YOff = CalcLoc.getNorthY() - GPSVal.getNorthY();
		double DistOff = Math.sqrt(XOff * XOff + YOff * YOff);

		System.out.println("DistOff: " + DistOff);
		// For every meter we're off target, add 5 particles (randomly in a
		// radius about GPSVal, at half radius of off target)
		// If we're within a meter, subtract particles, killing duplicates (down
		// to one)
		if (DistOff > 1.2) {
			int NewParts = (int) (DistOff * 5.0);
			int NewMixParts = NewParts / 20;
			Matrix tempParticle = new Matrix(3, 1);
			for (int i = 0; i < NewParts; i++) {
				tempParticle = new Matrix(3, 1);
				Double RandRadius = (((DistOff / 2) * rand.nextDouble()) * (-1 * rand.nextInt(2)));
				tempParticle.set(0, 0, GPSVal.getEastX() + RandRadius);
				RandRadius = (((DistOff / 2) * rand.nextDouble()) * (-1 * rand.nextInt(2)));
				tempParticle.set(1, 0, GPSVal.getNorthY() + RandRadius);
				RandRadius = ((1 * rand.nextDouble()) * (-1 * rand.nextInt(2)));
				tempParticle.set(2, 0, RandRadius);
				Outgoing.add(tempParticle);
			}
			NumParticles = NumParticles + NewParts;
		} else {
			// Within a Meter
			int RemLoc;
			if (NumParticles > 20) {
				// Make sure we have particles to remove

				// Remove all particles with lowest possible weight
				// RemLoc = iWeights.indexOf(0.0);
				// while (RemLoc > -1) {
				// // //DEBUG
				// // System.out.println("RemLoc = " + RemLoc);
				// // //ENDDEBUG
				// Outgoing.remove(RemLoc);
				// iWeights.remove(RemLoc);
				// NumParticles--;
				// RemLoc = iWeights.indexOf(0.0);
				// }
				// Remove all particles with a weight less than CullPoint
				double CullPoint = 0.1;
				for (int i = 0; i < NumParticles; i++) {
					if (iWeights.get(i) < CullPoint) {
						Outgoing.remove(i);
						iWeights.remove(i);
						NumParticles--;
					}
				}

			} else {
				System.out.println("Can't cull a low pop.");
			}

		}

		return Outgoing;
	}

	private Vector<Matrix> LowVarianceResample(Vector<Matrix> Incoming, Vector<Double> iWeights, Double TWeight) {
		int M = Incoming.size();
		// System.out.println("Incoming Size: "+M);
		// System.out.println("iWeights Size:" + iWeights.size());
		Double r = rand.nextDouble() * (1.0 / (double) M);
		// System.out.println("r: " + r);
		Double c = (iWeights.get(0) / TWeight);
		Double U;
		int i = 1;
		Vector<Matrix> Outgoing = new Vector<Matrix>();
		for (int m = 1; m <= M; m++) {
			U = r + (((double) m - 1.0) / (double) M);
			// System.out.println("U = " + U);
			// System.out.println("m = " + m);
			while (U > c) {
				// //DEBUG
				// System.out.println("i in LVR: " + i);
				// System.out.println("c @ i = "+ c);
				// //END DEBUG
				i++;
				c += (iWeights.get(i - 1) / TWeight);
			}
			Outgoing.add(Incoming.get(i - 1));
		}
		// System.out.println("Outgoing Size: "+Outgoing.size());

		return Outgoing;
	}

	private Vector<Matrix> WRS(Vector<Matrix> Incoming, Vector<Double> iWeights, Double TWeight) {
		Vector<Matrix> Outgoing = new Vector<Matrix>();
		int k = Incoming.size();

		for (int i = 0; i < k; i++) {
			// New Sampling Algorithm... faster, harder, funner?
			Double ki = rand.nextDouble();
			ki = Math.pow(ki, (1 / iWeights.get(0)));
			Double Tw = ki;
			Double RunWeight = 0.0;
			int Choice = 0;
			for (int j = 0; j < k; j++) {
				Double r = rand.nextDouble();
				Double Xw = Math.log(r) / Math.log(Tw);
				if (RunWeight < Xw && Xw <= (RunWeight + iWeights.get(j))) {
					Choice = j;
					Double tw = Math.pow(Tw, iWeights.get(j));
					Double r2 = rand.nextDouble();
					while (r2 <= tw) {
						r2 = rand.nextDouble();
					}
					Tw = Math.pow(r2, (1 / iWeights.get(j)));
				} else {
					RunWeight = RunWeight + iWeights.get(j);
				}
			}
			// // DEBUG
			// System.out.println("Choice for LowVarianceResample: " + Choice);
			// // ENDDEBUG
			Outgoing.add(Incoming.get(Choice));

		}

		return Outgoing;
	}

	private Vector<Matrix> KLD(Vector<Matrix> Incoming, GPSEntry NewGPS, Vector<Double> iWeights, Double TWeight) {
		Vector<Matrix> Outgoing = new Vector<Matrix>();
		int iSize = Incoming.size();
		Double NewX = NewGPS.getEastX();
		Double NewY = NewGPS.getNorthY();

		// //DEBUG
		// NumKLD++;
		// System.out.println("NumParticles Pre KLD: "+NumParticles);
		// //END DEBUG

		// Build B about GPS - 0.5m x 0.5m x pi/12 rad out to 10 m
		// Starts @ East (Y = 0, X = 0.5n m) and rotates CCW by pi/12 rad
		// intervals.
		Vector<Integer> B = new Vector<Integer>();
		for (int i = 0; i < 480; i++) {
			B.add(0);
		}
		int M = 0;
		int Mx = (int) ((double) iSize * 1.5);// 100000;
		int k = 0;
		Double z = 2.33;
		Double e = 0.05;

		do {
			Double ki = rand.nextDouble();
			ki = Math.pow(ki, (1 / iWeights.get(0)));
			Double Tw = ki;
			Double RunWeight = 0.0;
			int Choice = 0;
			for (int j = 0; j < iSize; j++) {
				Double r = rand.nextDouble();
				Double Xw = Math.log(r) / Math.log(Tw);
				if (RunWeight < Xw && Xw <= (RunWeight + iWeights.get(j))) {
					Choice = j;
					Double tw = Math.pow(Tw, iWeights.get(j));
					Double r2 = rand.nextDouble();
					while (r2 <= tw) {
						r2 = rand.nextDouble();
					}
					Tw = Math.pow(r2, (1 / iWeights.get(j)));
				} else {
					RunWeight = RunWeight + iWeights.get(j);
				}
			}

			Matrix tempParticle = Incoming.get(Choice);
			Outgoing.add(tempParticle);
			// Solve for binning
			Double TempX = NewX - tempParticle.get(0, 0);
			Double TempY = NewY - tempParticle.get(1, 0);
			Vector<Double> tempPolar = CartToPolar(TempX, TempY);
			Double TempRads = tempPolar.get(1);
			if (TempY < 0)
				TempRads += Math.PI;
			Double TempDist = tempPolar.get(0);
			int BinNum = 0;
			int BinRing = 0;
			int BinRad = 0;
			if (TempDist > 10) {
				// Outside our 10 meter range, max bin radius
				BinRing = 19;
			} else {
				BinRing = (int) (TempDist / 0.5);
			}
			BinNum = (BinRing * 24);
			BinRad = (int) (TempRads / (Math.PI / 12));
			BinNum += BinRad;
			// System.out.println("BinNum: " + BinNum);
			if (B.get(BinNum) == 0) {
				// Bin is Empty!
				// System.out.println("Bin is Empty");
				k++;
				B.set(BinNum, 1);
				if (k > 1) {
					Mx = (int) (((k - 1) / (2 * e)) * Math.pow((1 - (2 / (9 * (k - 1))) + (Math.sqrt(2 / (9 * (k - 1))) * z)), 3));
					// System.out.println("Mx Update: " + Mx);
				}
			}
			M = M + 1;

		} while (M < Mx);

		NumParticles = Outgoing.size();
//		// DEBUG
//		System.out.println("NumParticles Post KLD: " + NumParticles);
//		// try {
//		// BufferedWriter out = new BufferedWriter(new FileWriter(NumKLD +
//		// ".csv"));
//		// boolean FirstLine = true;
//		// if (NumKLD > 1) {
//		// BufferedReader in = new BufferedReader(new FileReader((NumKLD - 1) +
//		// ".csv"));
//		// String TempStr;
//		// int i = 0;
//		// while ((TempStr = in.readLine()) != null) {
//		// if (i < NumParticles) {
//		// if (FirstLine) {
//		// FirstLine = false;
//		// out.write(TempStr + ",X" + NumKLD + ",Y" + NumKLD+",");
//		// } else
//		// out.write(TempStr + "," + Outgoing.get(i).get(0, 0) + "," +
//		// Outgoing.get(i).get(1, 0)+",");
//		// } else {
//		// out.write(TempStr + ",,,");
//		// }
//		// i++;
//		// out.newLine();
//		// }
//		//
//		// for (; i < NumParticles; i++) {
//		// for (int j = 0; j < NumKLD; j++) {
//		// out.write(",,,");
//		// }
//		// out.write(Outgoing.get(i).get(0, 0) + "," + Outgoing.get(i).get(1,
//		// 0)+",");
//		// out.newLine();
//		// }
//		// } else {
//		// for (int i = 0; i < NumParticles; i++) {
//		// if (FirstLine) {
//		// FirstLine = false;
//		// out.write("X" + NumKLD + ",Y" + NumKLD + ",");
//		// } else
//		// out.write(Outgoing.get(i).get(0, 0) + "," + Outgoing.get(i).get(1,
//		// 0)+",");
//		// out.newLine();
//		// }
//		// }
//		// out.close();
//		// } catch (IOException KLDOutIOExc) {
//		// System.out.println("Output in KLD Blew Up");
//		// KLDOutIOExc.printStackTrace();
//		// }
//		//
//		// END DEBUG
		return Outgoing;
	}

	private Vector<Double> PolarToCart(Double Dist, Double Rads) {
		Vector<Double> Cart = new Vector<Double>();
		// polar to Cartesian
		double x = Math.cos(Rads) * Dist;
		double y = Math.sin(Rads) * Dist;

		Cart.add(x);
		Cart.add(y);

		return Cart;
	}

	private Vector<Double> CartToPolar(Double X, Double Y) {
		Vector<Double> Pole = new Vector<Double>();
		// Cartesian to polar.
		double radius = Math.sqrt(X * X + Y * Y);
		double angleInRadians = Math.acos(X / radius);

		Pole.add(radius);
		Pole.add(angleInRadians);

		return Pole;
	}

	private class SPKFStep {
		private Matrix P;

		private Matrix X;

		private Matrix Residual;

		private Double dTimestep;

		private Vector<Double> dControl;

		private Vector<Double> dMeasure;

		private Vector<Double> dGreek;

		private boolean StepDone = false;

		void setP(Matrix PDash) {
			P = PDash;
		}

		void setX(Matrix XDash) {
			X = XDash;
		}

		Matrix getP() {
			return P;
		}

		Matrix getX() {
			return X;
		}

		Matrix getResid() {
			return Residual;
		}

		Vector<Double> getGreek() {
			return dGreek;
		}

		boolean StepFinished() {
			return StepDone;
		}

		SPKFStep(Matrix PDash, Matrix XDash, Vector<Double> Control, Vector<Double> Measurement, Vector<Double> Greek, Double dInTimestep) {
			P = PDash;
			X = XDash;
			dGreek = Greek;
			dTimestep = dInTimestep;
			dControl = Control;
			dMeasure = Measurement;
			RunSPKF();
		}

		private Matrix SqrtSPKF(Matrix PDash) {
			// Only works with a symmetric Positive Definite matrix
			// [V,D] = eig(A)
			// S = V*diag(sqrt(diag(D)))*V'
			//			
			// //PDash in
			// System.out.println("PDash: ");
			// PDash.print(3, 2);

			// Matrix NegKeeper = Matrix.identity(9, 9);
			// //Testing Forced Compliance
			// for(int i = 0; i< 9; i++){
			// if (PDash.get(i, i)< 0){
			// NegKeeper.set(i,i,-1);
			// PDash.set(i, i, -PDash.get(i, i));
			// }
			// }
			EigenvalueDecomposition eig = PDash.eig();
			Matrix V = eig.getV();
			Matrix D = eig.getD();
			int iDSize = D.getRowDimension();
			for (int i = 0; i < iDSize; i++) {
				D.set(i, i, Math.sqrt(D.get(i, i)));
			}
			Matrix S = V.times(D).times(V.inverse());
			// S = S.times(NegKeeper);
			return S;
		}

		private void RunSPKF() {
			// SPKF Steps:
			// 1) Generate Test Points
			// 2) Propagate Test Points
			// 3) Compute Predicted Mean and Covariance
			// 4) Compute Measurements
			// 5) Compute Innovations and Cross Covariance
			// 6) Compute corrections and update

			// Line up initial variables from the controller!
			Double dAlpha = dGreek.get(0);
			Double dBeta = dGreek.get(1);
			cUController.setAlpha(dAlpha);
			cUController.setBeta(dBeta);
			cUController.setKappa(dGreek.get(2));
			Double dGamma = cUController.getGamma();
			Double dLambda = cUController.getLambda();

			// // DEBUG - Print the Greeks
			// System.out.println("Greeks!");
			// System.out.println("Alpha - " + dAlpha);
			// System.out.println("Beta - " + dBeta);
			// System.out.println("Kappa - " + dGreek.get(2));
			// System.out.println("Lambda - " + dLambda);
			// System.out.println("Gamma - " + dGamma);

			// Let's get started:
			// Step 1: Generate Test Points
			Vector<Matrix> Chi = new Vector<Matrix>();
			Vector<Matrix> UpChi = new Vector<Matrix>();
			Vector<Matrix> UpY = new Vector<Matrix>();
			Matrix UpPx = new Matrix(3, 3, 0.0);
			Matrix UpPy = new Matrix(3, 3, 0.0);
			Matrix UpPxy = new Matrix(3, 3, 0.0);
			Matrix K;
			Vector<Double> wc = new Vector<Double>();
			Vector<Double> wm = new Vector<Double>();
			Chi.add(X); // Add Chi_0 - the current state estimate (X, Y, Z)

			// Big P Matrix is LxL diagonal
			Matrix SqrtP = SqrtSPKF(P);
			SqrtP = SqrtP.times(dGamma);

			// Set up Sigma Points
			for (int i = 0; i <= 8; i++) {
				Matrix tempVec = SqrtP.getMatrix(0, 8, i, i);
				Matrix tempX = X;
				Matrix tempPlus = tempX.plus(tempVec);
				// System.out.println("TempPlus");
				// tempPlus.print(3, 2);
				Matrix tempMinu = tempX.minus(tempVec);
				// System.out.println("TempMinus");
				// tempMinu.print(3, 2);
				// tempX = X.copy();
				// tempX.setMatrix(i, i, 0, 2, tempPlus);
				Chi.add(tempPlus);
				// tempX = X.copy();
				// tempX.setMatrix(i, i, 0, 2, tempMinu);
				Chi.add(tempMinu);
			}

			// DEBUG Print the lines inside the Chi Matrix (2L x L)
			// for (int i = 0; i<=(2*L); i++){
			// System.out.println("Chi Matrix Set: "+i);
			// Chi.get(i).print(5, 2);
			// }

			// Generate weights
			Double WeightZero = (dLambda / (L + dLambda));
			Double OtherWeight = (1 / (2 * (L + dLambda)));
			Double TotalWeight = WeightZero;
			wm.add(WeightZero);
			wc.add(WeightZero + (1 - (dAlpha * dAlpha) + dBeta));
			for (int i = 1; i <= (2 * L); i++) {
				TotalWeight = TotalWeight + OtherWeight;
				wm.add(OtherWeight);
				wc.add(OtherWeight);
			}
			// Weights MUST BE 1 in total
			for (int i = 0; i <= (2 * L); i++) {
				wm.set(i, wm.get(i) / TotalWeight);
				wc.set(i, wc.get(i) / TotalWeight);
			}

			// //DEBUG Print the weights
			// System.out.println("Total Weight:");
			// System.out.println(TotalWeight);
			// for (int i = 0; i<=(2*L); i++){
			// System.out.println("Weight M for "+i+" Entry");
			// System.out.println(wm.get(i));
			// System.out.println("Weight C for "+i+" Entry");
			// System.out.println(wc.get(i));
			// }

			// Step 2: Propagate Test Points
			// This will also handle computing the mean
			Double ux = dControl.elementAt(0);
			Double uy = dControl.elementAt(1);
			Double uz = dControl.elementAt(2);
			Matrix XhatMean = new Matrix(3, 1, 0.0);
			for (int i = 0; i < Chi.size(); i++) {
				Matrix ChiOne = Chi.get(i);
				Matrix Chixminus = new Matrix(3, 1, 0.0);
				Double Xhat = ChiOne.get(0, 0);
				Double Yhat = ChiOne.get(1, 0);
				Double Zhat = ChiOne.get(2, 0);
				Double Xerr = ChiOne.get(3, 0);
				Double Yerr = ChiOne.get(4, 0);
				Double Zerr = ChiOne.get(5, 0);

				Xhat = Xhat + ux + Xerr;
				Yhat = Yhat + uy + Yerr;
				Zhat = Zhat + uz + Zerr;

				Chixminus.set(0, 0, Xhat);
				Chixminus.set(1, 0, Yhat);
				Chixminus.set(2, 0, Zhat);
				// System.out.println("ChixMinus:");
				// Chixminus.print(3, 2);
				UpChi.add(Chixminus);
				XhatMean = XhatMean.plus(Chixminus.times(wm.get(i)));
			}

			// Mean is right!

			// System.out.println("XhatMean: ");
			// XhatMean.print(3, 2);

			// Step 3: Compute Predicted Mean and Covariance
			// Welp, we already solved the mean - let's do the covariance now
			for (int i = 0; i <= (2 * L); i++) {
				Matrix tempP = UpChi.get(i).minus(XhatMean);
				Matrix tempPw = tempP.times(wc.get(i));
				tempP = tempPw.times(tempP.transpose());
				UpPx = UpPx.plus(tempP);
			}

			// New Steps!

			// Step 4: Compute Measurements! (and Y mean!)
			Matrix YhatMean = new Matrix(3, 1, 0.0);
			for (int i = 0; i <= (2 * L); i++) {
				Matrix ChiOne = Chi.get(i);
				Matrix Chiyminus = new Matrix(3, 1, 0.0);
				Double Xhat = UpChi.get(i).get(0, 0);
				Double Yhat = UpChi.get(i).get(1, 0);
				Double Zhat = UpChi.get(i).get(2, 0);
				Double Xerr = ChiOne.get(6, 0);
				Double Yerr = ChiOne.get(7, 0);
				Double Zerr = ChiOne.get(8, 0);

				Xhat = Xhat + Xerr;
				Yhat = Yhat + Yerr;
				Zhat = Zhat + Zerr;

				Chiyminus.set(0, 0, Xhat);
				Chiyminus.set(1, 0, Yhat);
				Chiyminus.set(2, 0, Zhat);
				UpY.add(Chiyminus);
				YhatMean = YhatMean.plus(Chiyminus.times(wm.get(i)));
			}

			// // Welp, we already solved the mean - let's do the covariances
			// now
			// System.out.println("XHatMean and YHatMean = ");
			// XhatMean.print(3, 2);
			// YhatMean.print(3, 2);

			for (int i = 0; i <= (2 * L); i++) {
				Matrix tempPx = UpChi.get(i).minus(XhatMean);
				Matrix tempPy = UpY.get(i).minus(YhatMean);
				// System.out.println("ChiX - XhatMean and ChiY-YhatMean");
				// tempPx.print(3, 2);
				// tempPy.print(3, 2);

				Matrix tempPxw = tempPx.times(wc.get(i));
				Matrix tempPyw = tempPy.times(wc.get(i));

				tempPx = tempPxw.times(tempPy.transpose());
				tempPy = tempPyw.times(tempPy.transpose());
				UpPy = UpPy.plus(tempPy);
				UpPxy = UpPxy.plus(tempPx);
			}

			// Step 6: Compute Corrections and Update

			// Compute Kalman Gain!
			// System.out.println("Updated Px");
			// UpPx.print(5, 2);
			// System.out.println("Updated Py");
			// UpPy.print(5, 2);
			// System.out.println("Updated Pxy");
			// UpPxy.print(5, 2);
			K = UpPxy.times(UpPy.inverse());
			// System.out.println("Kalman");
			// K.print(5, 2);

			Matrix Mea = new Matrix(3, 1, 0.0);
			Mea.set(0, 0, dMeasure.get(0));
			Mea.set(1, 0, dMeasure.get(1));
			Mea.set(2, 0, dMeasure.get(2));

			Matrix Out = K.times(Mea.minus(YhatMean));
			Out = Out.plus(XhatMean);
			// System.out.println("Out:");
			// Out.print(3, 2);

			Matrix Px = UpPx.minus(K.times(UpPy.times(K.transpose())));

			// Update Stuff!
			// Push the P to the controller
			Matrix OutP = P.copy();
			OutP.setMatrix(0, 2, 0, 2, Px);
			X.setMatrix(0, 2, 0, 0, Out);

			Residual = XhatMean.minus(Out);
			cUController.inputState(OutP, Residual);
			// cController.setL(L);

			cUController.startProcess();
			while (!cUController.finishedProcess()) {
				try {
					Thread.sleep(10);
				} catch (InterruptedException e) {
					e.printStackTrace();
				}
			}

			dGreek.set(0, cUController.getAlpha());
			dGreek.set(1, cUController.getBeta());
			dGreek.set(2, cUController.getKappa());
			P = cUController.getP();

			StepDone = true;

		}

	}

	private Matrix KStep(Matrix vOldState, Vector<Double> dMeasurement, Vector<Double> dU, Double dTimeStep) {
		// Big Kalman - individual dR and dQ
		// Vector<Vector<Double>> vOutput = new Vector<Vector<Double>>();

		// Extract all of the necessary variables
		Double XKMinus = vOldState.get(0, 0);
		Double YKMinus = vOldState.get(0, 1);
		Double ZKMinus = vOldState.get(0, 2);
		Double XPkminus = vOldState.get(2, 0);
		Double YPkminus = vOldState.get(2, 1);
		Double ZPkminus = vOldState.get(2, 2);
		Double dQx = vOldState.get(3, 0);
		Double dQy = vOldState.get(3, 1);
		Double dQz = vOldState.get(3, 2);
		Double dRx = vOldState.get(4, 0);
		Double dRy = vOldState.get(4, 1);
		Double dRz = vOldState.get(4, 2);
		// Double alphax = vOldState.get(5,0);
		// Double alphay = vOldState.get(5,1);
		// Double alphaz = vOldState.elementAt(5).elementAt(2);

		// Project the New State ahead (drive!)
		Double XMinus = XKMinus + dU.get(0);
		Double YMinus = YKMinus + dU.get(1);
		Double ZMinus = ZKMinus + dU.get(2);
		// Double ZMinus = ZKMinus + dStateUpdate[3] * dTimeStep;

		// Project the Error Covariance Ahead
		Double XPMinus = XPkminus + dQx;
		Double YPMinus = YPkminus + dQy;
		Double ZPMinus = ZPkminus + dQz;
		// Double ZPMinus = alphaz * alphaz * (ZPkminus * alphaz * alphaz) +
		// dQz;

		// Compute Kalman Gain
		Double XKal = XPMinus / (XPMinus + dRx);
		Double YKal = YPMinus / (YPMinus + dRy);
		Double ZKal = ZPMinus / (ZPMinus + dRz);
		// Double ZKal = ZPMinus / (ZPMinus + dRz / (alphaz * alphaz));

		// Update Estimate with measurement
		Double X = XMinus + (XKal * (dMeasurement.get(0) - XMinus));
		Double Y = YMinus + (YKal * (dMeasurement.get(1) - YMinus));
		Double Z = ZMinus + (ZKal * (dMeasurement.get(2) - ZMinus));

		// Update the Error Covariance
		Double PX = (1 - XKal) / XPMinus;
		Double PY = (1 - YKal) / YPMinus;
		Double PZ = (1 - ZKal) / ZPMinus;

		// Output
		Matrix Out = vOldState;
		Out.set(0, 0, X);
		Out.set(0, 1, Y);
		Out.set(0, 2, Z);
		Out.set(1, 0, XMinus);
		Out.set(1, 1, YMinus);
		Out.set(1, 2, ZMinus);
		Out.set(2, 0, PX);
		Out.set(2, 1, PY);
		Out.set(2, 2, PZ);

		// Vector<Double> RowZero = new Vector<Double>();
		// RowZero.add(X);
		// RowZero.add(Y);
		// // RowZero.add(Z);
		// // NOT OUTPUT... =0
		// Vector<Double> RowOne = new Vector<Double>();
		// RowOne.add(XMinus);
		// RowOne.add(YMinus);
		// // RowOne.add(ZMinus);
		// Vector<Double> RowTwo = new Vector<Double>();
		// RowTwo.add(PX);
		// RowTwo.add(PY);
		// // RowTwo.add(PZ);
		//
		// vOutput.add(RowZero);
		// vOutput.add(RowOne);
		// vOutput.add(RowTwo);
		// vOutput.add(vOldState.elementAt(3));
		// vOutput.add(vOldState.elementAt(4));
		// vOutput.add(vOldState.elementAt(5));

		// Update alpha and R/Q
		cEController.inputState(Out);

		cEController.startProcess();
		while (!cEController.finishedProcess()) {
			try {
				Thread.sleep(10);
			} catch (InterruptedException e) {
				e.printStackTrace();
			}
		}
		Out.set(3, 0, cEController.getQ().get(0));
		Out.set(3, 1, cEController.getQ().get(1));
		Out.set(4, 0, cEController.getR().get(0));
		Out.set(4, 1, cEController.getR().get(1));
		Out.set(5, 0, cEController.getAlpha().get(0));
		Out.set(5, 1, cEController.getAlpha().get(1));

		// vOutput.remove(3);
		// vOutput.remove(3);
		// vOutput.remove(3);
		// vOutput.add(cEController.getQ());
		// vOutput.add(cEController.getR());
		// vOutput.add(cEController.getAlpha());

		return Out;
	}

	public Vector<GPSEntry> getOutput() {
		if (ProcessDone) {
			return Output;
		}
		return null;
	}

	public Vector<Matrix> getResiduals() {
		if (ProcessDone) {
			return Resid;
		}
		return null;
	}

}
