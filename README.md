
https://user-images.githubusercontent.com/44880102/123087762-e6a43680-d3f2-11eb-97cc-93e4feb01c51.mp4

# hal-scurve
hal scurve motion implementation

This project can be edited in 2 way's.
1. Qt creator, in qt open the project "qt-test.creator".
2. Text editor (geany), edit the "scurve.comp" file or edit the "curve.c" file.

When editing of the scurve.comp file is done, you can do a "halcompile --install" or a "halcompile --preprocess" to get the c file.
This c file can be used by qt to edit.

The scurve.c file can be compiled into a kernel module directly, use "halcompile --compile scurve.c".

The "runtest" is a file that is used by qt when compiling is done, it start the application. 

Quickstart surve in demo-mode:
  ```
  sudo halcompile --install scurve.comp  ( --compile, --preprocess )
	halcmd loadrt threads name1=base-thread fp1=0 period1=25000 name2=servo-thread period2=1000000
 	halcmd loadrt scurve
 	halcmd addf scurve servo-thread
 	halcmd setp scurve.demo-mode 1
 	halcmd start
	halscope    (oscilloscope program)
	halrun -U   (clean up hal)
  ```
  
Quickstart surve with parameter input:
  ```
	sudo halcompile --install scurve.comp  ( --compile, --preprocess )
	halcmd loadrt threads name1=base-thread fp1=0 period1=25000 name2=servo-thread period2=1000000
	halcmd loadrt scurve
	halcmd addf scurve servo-thread

	// Minimal input values :
	halcmd setp scurve.pathlenght 50      // requested pathlenght (can also be used as motor rpm when using this component as a softstarter)
	halcmd setp scurve.acc_lineair 2      // max acceleration value
	halcmd setp scurve.dcc_lineair 3      // max de-acceleration value
	halcmd setp scurve.vs 5               // max velocity

	// Extra input values :
	halcmd setp scurve.vo 0               // initial velocity
	halcmd setp scurve.ve 0               // end velocity

	halcmd setp scurve.acc_begin 0        // step into a curve with a initial acceleration.
	halcmd setp scurve.dcc_end 0          // stop the curve at a de-acceleration value. 

	halcmd setp scurve.time_stamp 1       // request scurve values at 1 sec.

	halcmd setp scurve.verbose 1          // verbose output 0 or 1.

	halcmd start
	halscope    (oscilloscope program)
	halmeter    (program to set and view hal pins)
	halrun -U   (exit, clean up hal)
  ```

Scurve outputs.

Return values at a certain time_stamp (seconds) :
	```
	pin out float r_displacment   	    	// displacement(s) of the scurve in mm.
	pin out float r_velocit			// velocity.
	pin out float r_acceleration  	    	// acceleration.
	pin out float r_jerk			// jerk
	```
Calculated time:
	```
	pin out float t1			// acceleration time.
	pin out float t2			// atspeed time, steady stage.
	pin out float t3			// de-acceleration time.
	pin out float ttot			// total time of path (T1+T2+T3).
	```
Calculated distance:
	```
	pin out float l1			// acceleration lenght.
	pin out float l2			// atspeed lenght, steady stage.
	pin out float l3			// de-acceleration lenght.
	pin out float ltot			// total path lenght (L1+L2+L3).
	```
Information about displacement(s) values:

	```
	pin out float acc_concave  		// displacement convave period acc, curve up, document page 1.
	pin out float acc_convex   		// displacement convex period acc, curve down.
	pin out float acc           		// displacement acc period, s_acc_concave + s_acc_convex.
	pin out float steady       		// displacement atspeed/steady period.
	pin out float dcc_convex   		// displacement convex period dcc, curve down. (one position for last curve segment).
	pin out float dcc_concave  		// displacement convave period dcc, curve up. (last curve segment).
	pin out float dcc          		// dispalcement dcc period, s_dcc_convex + s_dcc_concave".
	```
