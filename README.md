# Fermentation-Monitor

A non invasive, battery powered sourdough proofing monitor. It watches how much light passes through a jar of dough, cancels out room lighting and temperature effects, and classifies the dough as Rising, Peak, or Over proofed, so you do not have to poke, lift a cloth, or guess.

Built for ELEE 4230/6230 (Sensors and Transducers) at the University of Georgia, Summer 2026.


The problem

Home bakers usually judge proofing by eye and touch. The rise rate changes with flour, starter ratio, and kitchen temperature, and checking by disturbing the dough can deflate it or contaminate it. The goal of this project: determine rise progress non invasively, continuously, and accurately in an uncontrolled home kitchen.

How it works
A CdS photoresistor (LDR) measures how much light passes through the jar. As dough rises, the path gets more opaque.
A VEML7700 ambient light sensor measures room light so the algorithm can tell "the lights changed" apart from "the dough changed."
A PCT2075 temperature sensor measures the kitchen so the rise rate can be corrected for fermentation speed.
A homemade capacitive touch sensor (two foil plates with a plastic film between them) acts as the reset button. Pressing it marks "dough is in the jar, start tracking from here."
The Arduino filters each channel with a 1D Kalman filter, converts opacity to a rise percentage against a calibrated baseline, computes a temperature corrected slope, and classifies the fermentation stage.
