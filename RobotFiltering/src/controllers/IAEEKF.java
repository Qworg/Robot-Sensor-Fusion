package controllers;

import java.util.Vector;

import Jama.Matrix;

public class IAEEKF implements EKFControl {
	private Boolean Started = false;
	private Boolean Done = false;
	private Vector<Double> vdAlpha;
	private Vector<Double> vdQ;
	private Vector<Double> vdR;
	private Vector<Double> vdP;
	private Vector<Double> vdMinus;
	private Vector<Double> vdXYZ; 
	
	public IAEEKF() {
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
		if (Done){
			Done = false;
			Started = false;
			return !Done;			
		} 
		return Done;
	}

	public void inputState(Vector<Vector<Double>> State) {
		if(!Started) {
			vdXYZ = State.elementAt(0);
			vdMinus = State.elementAt(1);
			vdP = State.elementAt(2);
			vdQ = State.elementAt(3);
			vdR = State.elementAt(4);
			vdAlpha = State.elementAt(5);
		} else {
			System.out.println("Can't update the IAEEKF state while running!");
		}
	}

	public void startProcess() {
		//Oh, NOW you add things... =0
		if (!Started){
			Started = true;
			
			//EMPTY
			
			Done = true;
		} else {
			System.out.println("Can't update the IAEEKF state while running!");
		}
		
	}
	
	public void inputState(Matrix State) {
		if (!Started) {
			double[][] dState = State.getArray();
			vdXYZ.add(dState[0][0]);
			vdXYZ.add(dState[0][1]);
			vdXYZ.add(dState[0][2]);
			vdMinus.add(dState[1][0]);
			vdMinus.add(dState[1][1]);
			vdMinus.add(dState[1][2]);
			vdP.add(dState[2][0]);
			vdP.add(dState[2][1]);
			vdP.add(dState[2][2]);
			vdQ.add(dState[3][0]);
			vdQ.add(dState[3][1]);
			vdQ.add(dState[3][2]);
			vdR.add(dState[4][0]);
			vdR.add(dState[4][1]);
			vdR.add(dState[4][2]);
			vdAlpha.add(dState[5][0]);
			vdAlpha.add(dState[5][1]);
			vdAlpha.add(dState[5][2]);
		} else {
			System.out.println("Can't update the IAEEKF state while running!");
		}
	}

}
