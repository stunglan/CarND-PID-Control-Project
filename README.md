# My submission to the project in CarND-Controls-PID

Self-Driving Car Engineer Nanodegree Program


## PID controls
The PID control in it self could use the lecture more or less directly.

I implemented the derative control using time spend, but other that that its the same logic.

## Twiddle
I did not really understand how to adopt the twiddle routine directly, so I adjusted this somewhat to allow for a event loop, using the reset function in the simulator.

## Conclusion
The car wobbles a bit with my "twiddled" parameters, but keeps inside the road. If I decrease D controller somewhat, it still keeps inside the road, but are more gentle in the turns. 
