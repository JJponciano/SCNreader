# scn_reader
This is reader for scn file format

We want to detect a switch in the different cloud loaded.

We have achieved a first version of a programm which detect switch. But, it aplies only on the part of file. So, before to make statistic, we must extend the treatment on all the file, and we must ameliorate, clean it. For that, there are two main tasks:

==========================================EXPENDABLE======================================================

- Extend the treatment to apply on all file.
- Write in the text file footpulses which corresponding of detected switchs.
 
BE CAREFULL:
- Don't duplicate treatments.
- Use "ListeRail" as a window to do advance the treatment.

=========================================OPTIMIZATION=====================================================

- Remove the most possible of pointers.
- Test memory leak.
- Optimize algorithms, almost their complexity.
- Make secure prospective errors.
- Programming with multi-thread, to run quickly.

