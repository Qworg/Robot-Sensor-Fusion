package filters;

import java.io.BufferedWriter;
import java.io.FileWriter;
import java.io.IOException;
import java.util.Date;
import java.util.Vector;

import Jama.Matrix;

import testsystem.Data;
import testsystem.GPSEntry;
import controllers.Control;
import controllers.SPKFControl;

public class MHSPKF implements Filter {
	private int NumFilters = 0;

	private Vector<Filter> FilterList = new Vector<Filter>();
	
	private Vector<Vector<GPSEntry>> Outputs = new Vector<Vector<GPSEntry>>();
	
	private Vector<Vector<Matrix>> Resids = new Vector<Vector<Matrix>>();
	
	Vector<Matrix> Resid = new Vector<Matrix>();
	
	private Vector<Long> ElapsedTimes = new Vector<Long>();

	private boolean ProcessOn = false;

	private boolean ProcessDone = false;

	private Date TimeOn, TimeOff;

	private Long TimeElapsed;

	private Data dDataset;

	private Double dQ;

	private Double dR;

	private Double Alpha, Beta, Kappa;

	Vector<GPSEntry> Output = new Vector<GPSEntry>();

	public MHSPKF(Control Controller, Data Dataset, Double initQ, Double initR) {
		FilterList.add(new SPKF((SPKFControl) Controller, Dataset, initQ, initR));
		NumFilters++;
		dDataset = Dataset;
		dQ = initQ;
		dR = initR;
		Alpha = 0.02;
		Beta = 0.2;
		Kappa = 0.0;
	}

	public MHSPKF(Control Controller, Data Dataset, Double initQ, Double initR, Double iAlpha, Double iBeta, Double iKappa) {
		FilterList.add(new SPKF((SPKFControl) Controller, Dataset, initQ, initR, iAlpha, iBeta, iKappa));
		NumFilters++;
		dDataset = Dataset;
		dQ = initQ;
		dR = initR;
		Alpha = iAlpha;
		Beta = iBeta;
		Kappa = iKappa;
	}

	public Boolean finishedProcess() {
		return ProcessDone;
	}

	public void outputData(String FileName) {
		int iLen = FilterList.size();
		for (int j = 0; j < iLen; j++) {
			FilterList.elementAt(j).outputData(j + "-" + FileName);
		}
		try {
			BufferedWriter out = new BufferedWriter(new FileWriter(FileName+TimeElapsed+".txt"));
			int len = Output.size();
			GPSEntry A;
			Matrix B;
			out.write("#Timestep NorthX EastY ResidX ResidY ResidZ");
			out.newLine();
			for (int i = 0; i < len; i++) {
				A = Output.elementAt(i);
				B = Resid.elementAt(i);
				out.write(A.getGPSTimestamp() + " " + A.getEastX() + " " + A.getNorthY() + " " + B.get(0, 0) + " "+ B.get(0, 1) + " "+ B.get(0, 2));
				out.newLine();
			}
			out.close();
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}

	public void setControl(Control Controller) {
		// This differs in implementation from the other filters - it adds
		// multiple controllers to the same filter type, until they stop adding
		// them.
		if (!ProcessOn) {
			FilterList.add(new SPKF((SPKFControl) Controller, dDataset, dQ, dR));
			NumFilters++;
		} else {
			System.out.println("Can't add controllers to the MHSPKF while its running!");
		}

	}

	public void setControl(Control Controller, Double initQ, Double initR) {
		// This differs in implementation from the other filters - it adds
		// multiple controllers to the same filter type, until they stop adding
		// them.
		if (!ProcessOn) {
			FilterList.add(new SPKF((SPKFControl) Controller, dDataset, initQ, initR));
			NumFilters++;
		} else {
			System.out.println("Can't add controllers to the MHSPKF while its running!");
		}

	}

	public void setControl(Control Controller, Double initQ, Double initR, Double iAlpha, Double iBeta, Double iKappa) {
		// This differs in implementation from the other filters - it adds
		// multiple controllers to the same filter type, until they stop adding
		// them.
		if (!ProcessOn) {
			FilterList.add(new SPKF((SPKFControl) Controller, dDataset, initQ, initR, iAlpha, iBeta, iKappa));
			NumFilters++;
		} else {
			System.out.println("Can't add controllers to the MHSPKF while its running!");
		}

	}

	public void setData(Data Dataset) {
		dDataset = Dataset;
	}

	public void startProcess() {
		// Start the process!
		ProcessOn = true;
		TimeOn = new Date();
		// Get the length of the dataset
		int iLength = dDataset.getLength();

		Filter temp;
		for (int i = 0; i<NumFilters; i++){
			//Run each filter, glean results, make this happen, boogie.
			temp = FilterList.get(i);
			temp.startProcess();
			
			while (!temp.finishedProcess()) {
				try {
					Thread.sleep(10);
				} catch (InterruptedException e) {
					e.printStackTrace();
				}
			}
			Outputs.add(temp.getOutput());
			Resids.add(temp.getResiduals());
			ElapsedTimes.add(temp.getTimeElapsed());			
		}
		
		for (int i = 0; i<iLength; i++){
			//Crunch Resids across, Solve Weight
			Matrix TotalResid = new Matrix (3, 1, 0.0);
			Vector<Matrix> Weights = new Vector<Matrix>();
			Matrix TotalWeight = new Matrix(1, 3, 0.0);
			for (int j = 0; j<NumFilters; j++) {
				//Add up all of the Residuals
//				System.out.println("Resid "+j+" at "+i);
//				Resids.get(j).get(i).print(3, 2);
				TotalResid = TotalResid.plus(Resids.get(j).get(i));
			}
//			System.out.println("Total Resid");
//			System.out.println(TotalResid.get(0, 0) + "," + TotalResid.get(0, 1) + "," + TotalResid.get(0, 2));
			for (int j = 0; j<NumFilters; j++) {
				//Solve for the weights -> 1-Rn/Rt
				Matrix tempWeight = new Matrix (1, 3, 0.0);
				Matrix tempI = new Matrix (1, 3, 1.0);
				//tempWeight = Resids.get(j).get(i).times(TotalResid.inverse());
				tempWeight.set(0, 0, Resids.get(j).get(i).get(0, 0)/TotalResid.get(0, 0));
				tempWeight.set(0, 1, Resids.get(j).get(i).get(1, 0)/TotalResid.get(1, 0));
				tempWeight.set(0, 2, Resids.get(j).get(i).get(2, 0)/TotalResid.get(2, 0));
				tempWeight = tempI.minus(tempWeight);
				TotalWeight = TotalWeight.plus(tempWeight);
				Weights.add(tempWeight);
//				System.out.println("1-Rn/Rt");
//				tempWeight.print(3, 2);
			}
//			System.out.println("Total Weight");
//			TotalWeight.print(3, 2);
			Matrix Chi = new Matrix (1, 3, 0.0);
			for (int j = 0; j<NumFilters; j++) {
				//Solve for the final answer
				Matrix tempWeight = Weights.get(j); 
				Matrix tempChi = new Matrix (1, 3, 0.0);
				tempChi.set(0, 0, Outputs.get(j).get(i).getEastX()*tempWeight.get(0, 0));
				tempChi.set(0, 1, Outputs.get(j).get(i).getNorthY()*tempWeight.get(0, 1));
				//Add Z if necessary later
				Chi = Chi.plus(tempChi);
			}
			Matrix tempF = new Matrix (3, 1, 1.0);
			tempF = tempF.times(TotalWeight);
			Output.add(new GPSEntry(Chi.get(0, 0)/tempF.get(0, 0), Chi.get(0, 1)/tempF.get(0, 1), Outputs.get(0).get(i).getGPSTimestamp()));
			Matrix tempR = new Matrix (1, 3, 0.0);
			GPSEntry gGPS = dDataset.GetGPSEntry(i);
			tempR.set(0, 0, gGPS.getEastX()-Chi.get(0, 0)/tempF.get(0, 0));
			tempR.set(0, 1, gGPS.getNorthY()-Chi.get(0, 1)/tempF.get(0, 1));
			Resid.add(tempR);
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

	public Vector<GPSEntry> getOutput() {
		if (ProcessDone) {
			return Output;
		}
		return null;
	}

	public Vector<Matrix> getResiduals() {
		// TODO Auto-generated method stub
		return null;
	}
}
