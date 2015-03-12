# SurfReg
Automatic surface registration tool for [ICL-NUIM datasets](http://www.doc.ic.ac.uk/~ahanda/VaFRIC/iclnuim.html).

Requires PCL 1.7 and Boost.

Your input reconstruction should be aligned with the position/orientation of the first frame of the sequence at the origin with the originally defined camera parameters (i.e., you must use the negative focal length). If your reconstruction's coordinate system is correctly aligned to the first frame of the sequence, automatic registration/evaluation will succeed. 

Parameters:
* -r \<reconstructed point cloud in ply format\>
* -m \<ground truth model file, available [here](http://www.doc.ic.ac.uk/~ahanda/living-room.ply.tar.gz)\>
* -t \<dataset id number (0-3), available [here](http://www.doc.ic.ac.uk/~ahanda/VaFRIC/iclnuim.html)\>

Run like;

```bash
./SurfReg -r reconstruction.ply -m living-room.ply -t 0
```
