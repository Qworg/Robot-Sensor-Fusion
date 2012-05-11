package controllers;

import java.util.Vector;

import Jama.Matrix; //Import the Fuzzy Logic system
import net.sourceforge.jFuzzyLogic.FIS;
import net.sourceforge.jFuzzyLogic.rule.FuzzyRuleSet;

public class FuzzyEKF implements EKFControl {
	private Vector<Double> vdAlpha = new Vector<Double>();

	private Vector<Double> vdQ = new Vector<Double>();

	private Vector<Double> vdR = new Vector<Double>();

	private Vector<Double> vdP = new Vector<Double>();

	private Vector<Double> vdMinus = new Vector<Double>();

	private Vector<Double> vdXYZ = new Vector<Double>();

	private Boolean Started = false;

	private Boolean Done = false;

	FIS fis = null;

	FuzzyRuleSet frs = null;

	public FuzzyEKF(String FN) {
		super();
		fis = FIS.load(FN);
		frs = fis.getFuzzyRuleSet();
	}

	public Vector<Double> getAlpha() {
		return vdAlpha;
	}

	public Vector<Double> getQ() {
		return vdQ;
	}

	public Vector<Double> getR() {
		return vdR;
	}

	public Boolean finishedProcess() {
		if (Done) {
			Done = false;
			Started = false;
			return !Done;
		}
		return Done;
	}

	public void inputState(Vector<Vector<Double>> State) {
		if (!Started) {
			vdXYZ = State.elementAt(0);
			vdMinus = State.elementAt(1);
			vdP = State.elementAt(2);
			vdQ = State.elementAt(3);
			vdR = State.elementAt(4);
			vdAlpha = State.elementAt(5);
		} else {
			System.out.println("Can't update the FuzzyEKF state while running!");
		}
	}

	public void startProcess() {
		if (!Started) {
			Started = true;
			Double X, Y, Z;
			Double XMinus, YMinus, ZMinus;
			Double PX, PY, PZ;
			Double dQx, dQy, dQz;
			Double dRx, dRy, dRz;
			Double alphax, alphay, alphaz;

			X = vdXYZ.elementAt(0);
			Y = vdXYZ.elementAt(1);
			Z = vdXYZ.elementAt(2);
			XMinus = vdMinus.elementAt(0);
			YMinus = vdMinus.elementAt(1);
			ZMinus = vdMinus.elementAt(2);
			PX = vdP.elementAt(0);
			PY = vdP.elementAt(1);
			PZ = vdP.elementAt(2);
			dQx = vdQ.elementAt(0);
			dQy = vdQ.elementAt(1);
			dQz = vdQ.elementAt(2);
			dRx = vdR.elementAt(0);
			dRy = vdR.elementAt(1);
			dRz = vdR.elementAt(2);
			alphax = vdAlpha.elementAt(0);
			alphay = vdAlpha.elementAt(1);
			alphaz = vdAlpha.elementAt(2);

			// Hopefully we don't have to break it apart by direction... BUT WE
			// DO!
			// Fuzzy Work
			// X
			frs.setVariable("covar", (PX / dRx));
			frs.setVariable("avg", (XMinus - X));
			frs.evaluate();
			alphax = frs.getVariable("alpha").defuzzify();
			// Y
			frs.setVariable("covar", (PY / dRy));
			frs.setVariable("avg", (YMinus - Y));
			frs.evaluate();
			alphay = frs.getVariable("alpha").defuzzify();
			// Z
			frs.setVariable("covar", (PZ / dRz));
			frs.setVariable("avg", (ZMinus - Z));
			frs.evaluate();
			alphaz = frs.getVariable("alpha").defuzzify();

			// Update alpha and R/Q
			dQx = dQx * alphax; // Math.pow(alphax, 4);
			dRx = dRx * alphax; // Math.pow(alphax, 4);
			dQy = dQy * alphay; // Math.pow(alphay, 4);
			dRy = dRy * alphay; // Math.pow(alphay, 4);
			dQz = dQz * alphaz; // Math.pow(alphaz, 4);
			dRz = dRz * alphaz; // Math.pow(alphaz, 4);

			// Sanity
			if (dQx > 100.0)
				dQx = 100.0;
			if (dQx < 0.001)
				dQx = 0.001;
			if (dQy > 100.0)
				dQy = 100.0;
			if (dQy < 0.001)
				dQy = 0.001;
			if (dQz > 100.0)
				dQz = 100.0;
			if (dQz < 0.001)
				dQz = 0.001;
			if (dRx > 100.0)
				dRx = 100.0;
			if (dRx < 0.001)
				dRx = 0.001;
			if (dRy > 100.0)
				dRy = 100.0;
			if (dRy < 0.001)
				dRy = 0.001;
			if (dRz > 100.0)
				dRz = 100.0;
			if (dRz < 0.001)
				dRz = 0.001;

			// DEBUG
			// TODO Remove this later
			// System.out.println("dQx, dQy, dQz");
			// System.out.println(dQx + "," +dQy + "," + dQz);
			// System.out.println("dRx, dRy, dRz");
			// System.out.println(dRx + "," +dRy + "," + dRz);

			vdQ.clear();
			vdR.clear();
			vdAlpha.clear();
			vdQ.add(dQx);
			vdQ.add(dQy);
			vdQ.add(dQz);
			vdR.add(dRx);
			vdR.add(dRy);
			vdR.add(dRz);
			// vdAlpha.add(alphax);
			// vdAlpha.add(alphay);
			// vdAlpha.add(alphaz);
			vdAlpha.add(1.0);
			vdAlpha.add(1.0);
			vdAlpha.add(1.0);

			Done = true;
		} else {
			System.out.println("Can't start the FuzzyEKF while its running!");
		}
	}

	public void inputState(Matrix State) {
		if (!Started) {
			// double[][] dState = State.getArray();
			vdXYZ.clear();
			vdMinus.clear();
			vdP.clear();
			vdQ.clear();
			vdR.clear();
			vdAlpha.clear();
			vdXYZ.add(State.get(0, 0));
			vdXYZ.add(State.get(0, 1));
			vdXYZ.add(State.get(0, 2));
			vdMinus.add(State.get(1, 0));
			vdMinus.add(State.get(1, 1));
			vdMinus.add(State.get(1, 2));
			vdP.add(State.get(2, 0));
			vdP.add(State.get(2, 1));
			vdP.add(State.get(2, 2));
			vdQ.add(State.get(3, 0));
			vdQ.add(State.get(3, 1));
			vdQ.add(State.get(3, 2));
			vdR.add(State.get(4, 0));
			vdR.add(State.get(4, 1));
			vdR.add(State.get(4, 2));
			vdAlpha.add(State.get(5, 0));
			vdAlpha.add(State.get(5, 1));
			vdAlpha.add(State.get(5, 2));
		} else {
			System.out.println("Can't update the IAEEKF state while running!");
		}
	}

}
