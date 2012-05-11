package controllers;

import java.util.Vector;

import Jama.Matrix;

public class NoControlEKF implements EKFControl {
	private Boolean Started = false;

	private Boolean Done = false;

	private Vector<Double> vdAlpha = new Vector<Double>();

	private Vector<Double> vdQ = new Vector<Double>();

	private Vector<Double> vdR = new Vector<Double>();

	// private Vector<Double> vdP;
	// private Vector<Double> vdMinus;
	// private Vector<Double> vdXYZ;

	public NoControlEKF() {
		super();
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
			// vdXYZ = State.elementAt(0);
			// vdMinus = State.elementAt(1);
			// vdP = State.elementAt(2);
			vdQ = State.elementAt(3);
			vdR = State.elementAt(4);
			vdAlpha = State.elementAt(5);
		} else {
			System.out.println("Can't update the NoControlEKF state while running!");
		}
	}

	public void startProcess() {
		// Not much in here!
		if (!Started) {
			Started = true;

			// EMPTY

			Done = true;
		} else {
			System.out.println("Can't update the NoControlEKF state while running!");
		}

	}

	public void inputState(Matrix State) {
		if (!Started) {
			//double[][] dState = State.getArray();
			//State.print(3,2);
			vdQ.clear();
			vdR.clear();
			vdAlpha.clear();
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
			System.out.println("Can't update the NoControlEKF state while running!");
		}
	}

}
