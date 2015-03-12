# SurfReg
Automatic surface registration tool for [ICL-NUIM datasets](http://www.doc.ic.ac.uk/~ahanda/VaFRIC/iclnuim.html).

Requires PCL 1.7 and Boost.

Parameters:
* -r \<reconstructed point cloud in ply format\>
* -m \<ground truth model file, available [here](http://www.doc.ic.ac.uk/~ahanda/living-room.ply.tar.gz)\>
* -t \<dataset id number (0-3), available [here](http://www.doc.ic.ac.uk/~ahanda/VaFRIC/iclnuim.html)\>

Run like;

```bash
./SurfReg -r reconstruction.ply -m living-room.ply -t 0
```
