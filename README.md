# scn_reader
This is reader for scn file format

We want to detect a switch in the different cloud loaded.

For detected a switch we should beginning with detected a track.
You can use a work windows for detectig the ground.
1)For detected a track, you could removed all of the points of the ground for each foot pulse to get only the local maximums.

1.bis)Then, if you want, you could extracted each local maximum in a cluster to keep only the average point of each cluster. 

2)You have more lines and each line could be a track. To determine which line is a rail, you would select a line two by two and you would test if the minimum distance between two point of this lines is the same, foot pulse after foot pulse.

3)Finaly,
