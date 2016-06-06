package movementModel;

import burlap.behavior.singleagent.EpisodeAnalysis;
import burlap.behavior.singleagent.learning.LearningAgent;
import burlap.behavior.singleagent.learning.tdmethods.QLearning;
import burlap.oomdp.auxiliary.DomainGenerator;
import burlap.oomdp.auxiliary.stateconditiontest.StateConditionTest;
import burlap.oomdp.core.*;
import burlap.oomdp.core.Attribute.AttributeType;
import burlap.oomdp.core.objects.MutableObjectInstance;
import burlap.oomdp.core.objects.ObjectInstance;
import burlap.oomdp.core.states.MutableState;
import burlap.oomdp.core.states.State;
import burlap.oomdp.core.values.Value;
import burlap.oomdp.singleagent.*;
import burlap.oomdp.singleagent.common.SimpleAction;
import burlap.oomdp.singleagent.environment.Environment;
import burlap.oomdp.singleagent.environment.SimulatedEnvironment;
import burlap.oomdp.singleagent.explorer.TerminalExplorer;
import burlap.oomdp.singleagent.explorer.VisualExplorer;
import burlap.oomdp.statehashing.HashableStateFactory;
import burlap.oomdp.visualizer.ObjectPainter;
import burlap.oomdp.visualizer.StateRenderLayer;
import burlap.oomdp.visualizer.StaticPainter;
import burlap.oomdp.visualizer.Visualizer;

import java.awt.*;
import java.awt.geom.Ellipse2D;
import java.awt.geom.Rectangle2D;
import java.util.*;
import java.util.List;

public class ExampleGridWorld implements DomainGenerator {
	
	public static final String ATTX = "x";
	public static final String ATTY = "y";
	public static final String ID = "id";
	public static final String DIR = "dir";

	public static final String CLASSAGENT = "agent";
	public static final String CLASSGROUPMATE = "groupMate";

	public static final String ACTION0 = "d0";
	public static final String ACTION10 = "d10";
	public static final String ACTION20 = "d20";
	public static final String ACTION30 = "d30";
	public static final String ACTION40 = "d40";
	public static final String ACTION50 = "d50";
	public static final String ACTION60 = "d60";
	public static final String ACTION70 = "d70";
	public static final String ACTION80 = "d80";
	public static final String ACTION90 = "d90";
	public static final String ACTION100 = "d100";
	public static final String ACTION110 = "d110";
	public static final String ACTION120 = "d120";
	public static final String ACTION130 = "d130";
	public static final String ACTION140 = "d140";
	public static final String ACTION150 = "d150";
	public static final String ACTION160 = "d160";
	public static final String ACTION170 = "d170";
	public static final String ACTION180 = "d180";
	public static final String ACTION190 = "d190";
	public static final String ACTION200 = "d200";
	public static final String ACTION210 = "d210";
	public static final String ACTION220 = "d220";
	public static final String ACTION230 = "d230";
	public static final String ACTION240 = "d240";
	public static final String ACTION250 = "d250";
	public static final String ACTION260 = "d260";
	public static final String ACTION270 = "d270";
	public static final String ACTION280 = "d280";
	public static final String ACTION290 = "d290";
	public static final String ACTION300 = "d300";
	public static final String ACTION310 = "d310";
	public static final String ACTION320 = "d320";
	public static final String ACTION330 = "d330";
	public static final String ACTION340 = "d340";
	public static final String ACTION350 = "d350";
	
	//ordered so first dimension is x
	protected int [][] map = new int[LANDSIZE][LANDSIZE];

	public static final int LANDSIZE = 51;

	
	/*
	public static void main(String [] args){

		ExampleGridWorld gen = new ExampleGridWorld();
		Domain domain = gen.generateDomain();

		State initialState = ExampleGridWorld.getExampleState(domain);
		
		int[] directionsGPS = new int[] {80,80,80,70,70,70,60,60,50, 50,50 ,50,40, 40, 40,30, 30, 30,20, 20,20, 10,10, 0, 0};
		RewardFunction rf = new DirectionRF(directionsGPS);
		TerminalFunction tf = new DirectionTF(directionsGPS.length);
		
		SimulatedEnvironment env = new SimulatedEnvironment(domain, rf, tf, initialState);

		//TerminalExplorer exp = new TerminalExplorer(domain, initialState);
		//exp.explore();

		Visualizer v = gen.getVisualizer();
		//VisualExplorer exp = new VisualExplorer(domain, v, initialState);
		VisualExplorer exp = new VisualExplorer(domain, env, v);

		exp.addKeyAction("w", ACTION0);
		exp.addKeyAction("s", ACTION90);
		exp.addKeyAction("d", ACTION180);
		exp.addKeyAction("a", ACTION350);

		exp.initGUI();
	}
*/
	
	/*************************************** Domain generation *************************************/

	@Override
	public Domain generateDomain() {
		SADomain domain = new SADomain();

		new Movement(ACTION0, domain, 0);
		new Movement(ACTION10, domain, 1);
		new Movement(ACTION20, domain, 2);
		new Movement(ACTION30, domain, 3);
		new Movement(ACTION40, domain, 4);
		new Movement(ACTION50, domain, 5);
		new Movement(ACTION60, domain, 6);
		new Movement(ACTION70, domain, 7);
		new Movement(ACTION80, domain, 8);
		new Movement(ACTION90, domain, 9);
		new Movement(ACTION100, domain, 10);
		new Movement(ACTION110, domain, 11);
		new Movement(ACTION120, domain, 12);
		new Movement(ACTION130, domain, 13);
		new Movement(ACTION140, domain, 14);
		new Movement(ACTION150, domain, 15);
		new Movement(ACTION160, domain, 16);
		new Movement(ACTION170, domain, 17);
		new Movement(ACTION180, domain, 18);
		new Movement(ACTION190, domain, 19);
		new Movement(ACTION200, domain, 20);
		new Movement(ACTION210, domain, 21);
		new Movement(ACTION220, domain, 22);
		new Movement(ACTION230, domain, 23);
		new Movement(ACTION240, domain, 24);
		new Movement(ACTION250, domain, 25);
		new Movement(ACTION260, domain, 26);
		new Movement(ACTION270, domain, 27);
		new Movement(ACTION280, domain, 28);
		new Movement(ACTION290, domain, 29);
		new Movement(ACTION300, domain, 30);
		new Movement(ACTION310, domain, 31);
		new Movement(ACTION320, domain, 32);
		new Movement(ACTION330, domain, 33);
		new Movement(ACTION340, domain, 34);
		new Movement(ACTION350, domain, 35);


		Attribute xatt = new Attribute(domain, ATTX, AttributeType.INT);
		xatt.setLims(0, LANDSIZE);

		Attribute yatt = new Attribute(domain, ATTY, AttributeType.INT);
		yatt.setLims(0, LANDSIZE);

		Attribute dir = new Attribute(domain, DIR, AttributeType.INT);
		dir.setLims(0, 36);

		Attribute id = new Attribute(domain, ID, AttributeType.INT);
		id.setLims(0, 40);

		ObjectClass agentClass = new ObjectClass(domain, CLASSAGENT);
		agentClass.addAttribute(xatt);
		agentClass.addAttribute(yatt);
		agentClass.addAttribute(dir);

		ObjectClass groupMateClass = new ObjectClass(domain, CLASSGROUPMATE);
		groupMateClass.addAttribute(xatt);
		groupMateClass.addAttribute(yatt);
		groupMateClass.addAttribute(id);



		return domain;
	}

	public static State getExampleState(Domain domain){ //could add time here to "move" the positions of the groupmates
		State s = new MutableState();
		ObjectInstance agent = new MutableObjectInstance(domain.getObjectClass(CLASSAGENT), "agent0");
		agent.setValue(ATTX, 25);
		agent.setValue(ATTY, 25);
		agent.setValue(DIR, 0);

		ObjectInstance groupmate = new MutableObjectInstance(domain.getObjectClass(CLASSGROUPMATE), 
				"groupMate0");
		groupmate.setValue(ATTX, 0);
		groupmate.setValue(ATTY, 30);
		groupmate.setValue(ID, 0);

		s.addObject(agent);
		s.addObject(groupmate);

		return s;
	}

	protected class Movement extends SimpleAction implements FullActionModel {

		//0: north
		protected double [] directionProbs = new double[36];

		public Movement(String actionName, Domain domain, int direction){

			super(actionName, domain);
			for(int i = 0; i < 36; i++){
				if(i == direction){
					directionProbs[i] = 1;
				}
				else{
					directionProbs[i] = 0;
				}
			}
		}

		protected int [] moveResult(int curX, int curY, int direction){

			int newDir = 0;

			if(direction == 0){
				newDir=0;
			} else if (direction == 1){
				newDir=10;
			} else if (direction == 2){
				newDir=20;
			} else if (direction == 3){
				newDir=30;
			} else if (direction == 4){
				newDir=40;
			} else if (direction == 5){
				newDir=50;
			} else if (direction == 6){
				newDir=60;
			} else if (direction == 7){
				newDir=70;
			} else if (direction == 8){
				newDir=80;
			} else if (direction == 9){
				newDir=90;
			} else if (direction == 10){
				newDir=100;
			} else if (direction == 11){
				newDir=110;
			} else if (direction == 12){
				newDir=120;
			} else if (direction == 13){
				newDir=130;
			} else if (direction == 14){
				newDir=140;
			} else if (direction == 15){
				newDir=150;
			} else if (direction == 16){
				newDir=160;
			} else if (direction == 17){
				newDir=170;
			} else if (direction == 18){
				newDir=180;
			} else if (direction == 19){
				newDir=190;
			} else if (direction == 20){
				newDir=200;
			} else if (direction == 21){
				newDir=210;
			} else if (direction == 22){
				newDir=220;
			} else if (direction == 23){
				newDir=230;
			} else if (direction == 24){
				newDir=240;
			} else if (direction == 25){
				newDir=250;
			} else if (direction == 26){
				newDir=260;
			} else if (direction == 27){
				newDir=270;
			} else if (direction == 28){
				newDir=280;
			} else if (direction == 29){
				newDir=290;
			} else if (direction == 30){
				newDir=300;
			} else if (direction == 31){
				newDir=310;
			} else if (direction == 32){
				newDir=320;
			} else if (direction == 33){
				newDir=330;
			} else if (direction == 34){
				newDir=340;
			} else if (direction == 35){
				newDir=350;
			}

			return new int[]{curX,curY,newDir};

		}



		@Override
		protected State performActionHelper(State s, GroundedAction groundedAction) {

			//get agent and current position
			ObjectInstance agent = s.getFirstObjectOfClass(CLASSAGENT);
			int curX = agent.getIntValForAttribute(ATTX);
			int curY = agent.getIntValForAttribute(ATTY);
			//int curDir = agent.getIntValForAttribute(DIR); assuming instantaneous change in direction of travel...

			
			//sample directon with random roll
			double r = Math.random();
			double sumProb = 0.;
			int dir = 0;
			for(int i = 0; i < this.directionProbs.length; i++){
				sumProb += this.directionProbs[i];
				if(r < sumProb){
					dir = i;
					break; //found direction
				}
			}

			//get resulting position
			int [] newPos = this.moveResult(curX, curY, dir);
			//System.out.println(newPos[2]);

			//set the new position
			agent.setValue(ATTX, newPos[0]);
			agent.setValue(ATTY, newPos[1]);
			agent.setValue(DIR, newPos[2]);

			//update the groupmates positions
			List<ObjectInstance> groupMates = s.getObjectsOfClass(CLASSGROUPMATE);

			for(ObjectInstance oo : groupMates){
				if(oo.getIntValForAttribute(ID)==0){
					oo.setValue(ATTX, Math.min(oo.getIntValForAttribute(ATTX)+1,LANDSIZE));

				}
			}


			//return the state we just modified
			return s;
		}

		@Override
		public List<TransitionProbability> getTransitions(State s, GroundedAction groundedAction) {

			//get agent and current position
			ObjectInstance agent = s.getFirstObjectOfClass(CLASSAGENT);
			int curX = agent.getIntValForAttribute(ATTX);
			int curY = agent.getIntValForAttribute(ATTY);
			int curDirection = agent.getIntValForAttribute(DIR);

			List<TransitionProbability> tps = new ArrayList<TransitionProbability>(36);
			TransitionProbability noChangeTransition = null;
			for(int i = 0; i < this.directionProbs.length; i++){
				int [] newPos = this.moveResult(curX, curY, i);
				if(newPos[0] != curX || newPos[1] != curY || newPos[2] !=curDirection){
					//new possible outcome
					State ns = s.copy();
					ObjectInstance nagent = ns.getFirstObjectOfClass(CLASSAGENT);
					nagent.setValue(ATTX, newPos[0]);
					nagent.setValue(ATTY, newPos[1]);

					//create transition probability object and add to our list of outcomes
					tps.add(new TransitionProbability(ns, this.directionProbs[i]));
				}
				else{
					//this direction didn't lead anywhere new
					//if there are existing possible directions
					//that wouldn't lead anywhere, aggregate with them
					if(noChangeTransition != null){
						noChangeTransition.p += this.directionProbs[i];
					}
					else{
						//otherwise create this new state and transition
						noChangeTransition = new TransitionProbability(s.copy(),
								this.directionProbs[i]);
						tps.add(noChangeTransition);
					}
				}
			}


			return tps;
		}

	}


	public class WallPainter implements StaticPainter{

		@Override
		public void paint(Graphics2D g2, State s, float cWidth, float cHeight) {

			//walls will be filled in black
			g2.setColor(Color.BLACK);

			//set up floats for the width and height of our domain
			float fWidth = ExampleGridWorld.this.map.length;
			float fHeight = ExampleGridWorld.this.map[0].length;

			//determine the width of a single cell on our canvas 
			//such that the whole map can be painted
			float width = cWidth / fWidth;
			float height = cHeight / fHeight;

			//pass through each cell of our map and if it's a wall, paint a black 
			//rectangle on our cavas of dimension widthxheight
			for(int i = 0; i < ExampleGridWorld.this.map.length; i++){
				for(int j = 0; j < ExampleGridWorld.this.map[0].length; j++){

					//is there a wall here?
					if(ExampleGridWorld.this.map[i][j] == 1){

						//left corrdinate of cell on our canvas
						float rx = i*width;

						//top coordinate of cell on our canvas
						//coordinate system adjustment because the java canvas
						//origin is in the top left instead of the bottom right
						float ry = cHeight - height - j*height;

						//paint the rectangle
						g2.fill(new Rectangle2D.Float(rx, ry, width, height));

					}


				}
			}

		}


	}

	public class AgentPainter implements ObjectPainter{

		@Override
		public void paintObject(Graphics2D g2, State s, ObjectInstance ob,
				float cWidth, float cHeight) {

			//agent will be filled in gray
			g2.setColor(Color.GRAY);

			//set up floats for the width and height of our domain
			float fWidth = ExampleGridWorld.this.map.length;
			float fHeight = ExampleGridWorld.this.map[0].length;

			//determine the width of a single cell on our canvas 
			//such that the whole map can be painted
			float width = cWidth / fWidth;
			float height = cHeight / fHeight;

			int ax = ob.getIntValForAttribute(ATTX);
			int ay = ob.getIntValForAttribute(ATTY);

			//left coordinate of cell on our canvas
			float rx = ax*width;

			//top coordinate of cell on our canvas
			//coordinate system adjustment because the java canvas 
			//origin is in the top left instead of the bottom right
			float ry = cHeight - height - ay*height;

			//paint the rectangle
			g2.fill(new Ellipse2D.Float(rx, ry, width, height));


		}



	}

	public class GroupMatePainter implements ObjectPainter{

		@Override
		public void paintObject(Graphics2D g2, State s, ObjectInstance ob,
				float cWidth, float cHeight) {

			//agent will be filled in blue
			g2.setColor(Color.BLUE);

			//set up floats for the width and height of our domain
			float fWidth = ExampleGridWorld.this.map.length;
			float fHeight = ExampleGridWorld.this.map[0].length;

			//determine the width of a single cell on our canvas 
			//such that the whole map can be painted
			float width = cWidth / fWidth;
			float height = cHeight / fHeight;

			int ax = ob.getIntValForAttribute(ATTX);
			int ay = ob.getIntValForAttribute(ATTY);

			//left coordinate of cell on our canvas
			float rx = ax*width;

			//top coordinate of cell on our canvas
			//coordinate system adjustment because the java canvas 
			//origin is in the top left instead of the bottom right
			float ry = cHeight - height - ay*height;

			//paint the rectangle
			g2.fill(new Rectangle2D.Float(rx, ry, width, height));


		}



	}

	/************************ rewards and termination ***********************************/

	public static class DirectionRF implements RewardFunction{

		int timeStep;
		int[] directionList; 

		public DirectionRF(int[] dList){
			directionList = dList;
			timeStep=0;
		}

		@Override
		public double reward(State s, GroundedAction a, State sprime) {

			//get direction of agent in next state
			ObjectInstance agent = sprime.getFirstObjectOfClass(CLASSAGENT);
			int direction = agent.getIntValForAttribute(DIR);

			//Correct direction chosen?
			if(directionList[timeStep] == direction){
				return 36.;
			}

			timeStep++;

			return -1;
		}


	}

	public static class DirectionTF implements TerminalFunction{

		int timeStep,timeStepMax;

		public DirectionTF(int dListSize){
			timeStepMax=dListSize;
			timeStep=0;
		}

		@Override
		public boolean isTerminal(State s) {

			//are they at goal time
			if(timeStep==timeStepMax-1){
				return true;
			}
			timeStep++;
			System.out.println("step "+timeStep+" : max = "+timeStepMax);
			return false;
		}



	}


	/************************ visualization *********************************************/

	public StateRenderLayer getStateRenderLayer(){
		StateRenderLayer rl = new StateRenderLayer();
		rl.addStaticPainter(new WallPainter());
		rl.addObjectClassPainter(CLASSGROUPMATE, new GroupMatePainter());
		rl.addObjectClassPainter(CLASSAGENT, new AgentPainter());

		return rl;
	}

	public Visualizer getVisualizer(){
		return new Visualizer(this.getStateRenderLayer());
	}
	
	/*****************************get and set **********************************************/
	
	public int[][] getMap(){
		return map;
	}


}