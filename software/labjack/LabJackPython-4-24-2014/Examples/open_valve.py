# An example script to show how to output a sine wave using a DAC.
# Because we have to do it all in software, there are limitations on how fast
# we can update the DAC. Update intervals faster than 5 ms may give weird
# results because of the large percentage of missed updates.
#
# Note: This example uses signal.setitimer() and signal.alarm(), and therefore 
# requires Python 2.6 on Unix to run. See:
#     http://docs.python.org/library/signal.html#signal.setitimer
#     http://docs.python.org/library/signal.html#signal.alarm
#
# When changing the update interval and frequency, consider how your values
# effect the waveform. A slow update interval coupled with a fast frequency
# can result in strange behavior. Try to keep the period (1/frequency) much
# greater than update interval.


# Constants. Change these to change the results:


# Imports:
import u3, u6, ue9 # For working with the U3

d = u3.U3()
#d.writeRegister(5000, 5)
d.writeRegister(5000, 0)


