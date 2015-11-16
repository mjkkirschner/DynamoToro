MODULE MainModule
	! Program data
	PERS tooldata tool:=[TRUE,[[10,0,0],[1,0,0,0]],[1,[0,0,0.001],[1,0,0,0],0,0,0]];

	TASK PERS wobjdata wobj:=[FALSE,TRUE,"",[[0,0,0],[1,0,0,0]],[[0,0,0],[1,0,0,0]]];


	! Target data
	CONST robtarget position0:=[[300,-10,100],[0,0,1,0],[0,0,0,0],[1E+10,1E+10,1E+10,1E+10,1E+10,1E+10]];
	CONST robtarget position1:=[[300,-9,100],[0,0,1,0],[0,0,0,0],[1E+10,1E+10,1E+10,1E+10,1E+10,1E+10]];
	CONST robtarget position2:=[[300,-8,100],[0,0,1,0],[0,0,0,0],[1E+10,1E+10,1E+10,1E+10,1E+10,1E+10]];
	CONST robtarget position3:=[[300,-7,100],[0,0,1,0],[0,0,0,0],[1E+10,1E+10,1E+10,1E+10,1E+10,1E+10]];
	CONST robtarget position4:=[[300,-6,100],[0,0,1,0],[0,0,0,0],[1E+10,1E+10,1E+10,1E+10,1E+10,1E+10]];
	CONST robtarget position5:=[[300,-5,100],[0,0,1,0],[0,0,0,0],[1E+10,1E+10,1E+10,1E+10,1E+10,1E+10]];
	CONST robtarget position6:=[[300,-4,100],[0,0,1,0],[0,0,0,0],[1E+10,1E+10,1E+10,1E+10,1E+10,1E+10]];
	CONST robtarget position7:=[[300,-3,100],[0,0,1,0],[0,0,0,0],[1E+10,1E+10,1E+10,1E+10,1E+10,1E+10]];
	CONST robtarget position8:=[[300,-2,100],[0,0,1,0],[0,0,0,0],[1E+10,1E+10,1E+10,1E+10,1E+10,1E+10]];
	CONST robtarget position9:=[[300,-1,100],[0,0,1,0],[0,0,0,0],[1E+10,1E+10,1E+10,1E+10,1E+10,1E+10]];
	CONST robtarget position10:=[[300,0,100],[0,0,1,0],[0,0,0,0],[1E+10,1E+10,1E+10,1E+10,1E+10,1E+10]];
	CONST robtarget position11:=[[300,1,100],[0,0,1,0],[0,0,0,0],[1E+10,1E+10,1E+10,1E+10,1E+10,1E+10]];
	CONST robtarget position12:=[[300,2,100],[0,0,1,0],[0,0,0,0],[1E+10,1E+10,1E+10,1E+10,1E+10,1E+10]];
	CONST robtarget position13:=[[300,3,100],[0,0,1,0],[0,0,0,0],[1E+10,1E+10,1E+10,1E+10,1E+10,1E+10]];
	CONST robtarget position14:=[[300,4,100],[0,0,1,0],[0,0,0,0],[1E+10,1E+10,1E+10,1E+10,1E+10,1E+10]];
	CONST robtarget position15:=[[300,5,100],[0,0,1,0],[0,0,0,0],[1E+10,1E+10,1E+10,1E+10,1E+10,1E+10]];
	CONST robtarget position16:=[[300,6,100],[0,0,1,0],[0,0,0,0],[1E+10,1E+10,1E+10,1E+10,1E+10,1E+10]];
	CONST robtarget position17:=[[300,7,100],[0,0,1,0],[0,0,0,0],[1E+10,1E+10,1E+10,1E+10,1E+10,1E+10]];
	CONST robtarget position18:=[[300,8,100],[0,0,1,0],[0,0,0,0],[1E+10,1E+10,1E+10,1E+10,1E+10,1E+10]];
	CONST robtarget position19:=[[300,9,100],[0,0,1,0],[0,0,0,0],[1E+10,1E+10,1E+10,1E+10,1E+10,1E+10]];
	CONST robtarget position20:=[[300,10,100],[0,0,1,0],[0,0,0,0],[1E+10,1E+10,1E+10,1E+10,1E+10,1E+10]];

	! Routine
	PROC main()
		ConfL\Off;
		SingArea\Wrist;
		rStart;
		RETURN;
	ENDPROC

	PROC rStart()

		! instructions
		MoveJ position0,v300,fine,tool\WObj:=wobj;
		MoveJ position1,v300,fine,tool\WObj:=wobj;
		MoveJ position2,v300,fine,tool\WObj:=wobj;
		MoveJ position3,v300,fine,tool\WObj:=wobj;
		MoveJ position4,v300,fine,tool\WObj:=wobj;
		MoveJ position5,v300,fine,tool\WObj:=wobj;
		MoveJ position6,v300,fine,tool\WObj:=wobj;
		MoveJ position7,v300,fine,tool\WObj:=wobj;
		MoveJ position8,v300,fine,tool\WObj:=wobj;
		MoveJ position9,v300,fine,tool\WObj:=wobj;
		MoveJ position10,v300,fine,tool\WObj:=wobj;
		MoveJ position11,v300,fine,tool\WObj:=wobj;
		MoveJ position12,v300,fine,tool\WObj:=wobj;
		MoveJ position13,v300,fine,tool\WObj:=wobj;
		MoveJ position14,v300,fine,tool\WObj:=wobj;
		MoveJ position15,v300,fine,tool\WObj:=wobj;
		MoveJ position16,v300,fine,tool\WObj:=wobj;
		MoveJ position17,v300,fine,tool\WObj:=wobj;
		MoveJ position18,v300,fine,tool\WObj:=wobj;
		MoveJ position19,v300,fine,tool\WObj:=wobj;
		MoveJ position20,v300,fine,tool\WObj:=wobj;
		RETURN;
	ENDPROC

ENDMODULE
