


![hal_oscilloscope](https://user-images.githubusercontent.com/44880102/123252140-f6d31900-d4b9-11eb-944d-afdf03cd37e4.jpg)

# hal-scurve
hal scurve motion implementation

This project can be edited in 2 way's.
1. Qt creator, in qt open the project "qt-test.creator".
2. Text editor (geany), edit the "scurve.comp" file or edit the "curve.c" file.

When editing of the scurve.comp file is done, you can do a "halcompile --install" or a "halcompile --preprocess" to get the c file.
This c file can be used by qt to edit.

The scurve.c file can be compiled into a kernel module directly, use "halcompile --compile scurve.c".

The "runtest" is a file that is used by qt when compiling is done, it load's the component, starts a oscilloscope etc. Easy for testing.

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

Return values at a certain time_stamp (seconds):
```
	pin out float r_displacment   	    	// displacement(s) of the scurve in mm.
	pin out float r_velocit			// velocity.
	pin out float r_acceleration  	    	// acceleration.
	pin out float r_jerk			// jerk
	pin out float r_trajecttime		// total traject time of scurve.
```

Implementation tip:
If you perform a scurve request with scurve.time-stamp=0. 
You get the hal pin value for: r_trajecttime (scurve traject time).
Then your "scurve.time-stamp" request can be done for the period : 
```
	0 >= scurve.time-stamp <= r_trajecttime
```

Running in demo-mode, "setp scurve.demo-mode 1"

https://user-images.githubusercontent.com/44880102/123252114-efac0b00-d4b9-11eb-869e-65afacd5e228.mp4

